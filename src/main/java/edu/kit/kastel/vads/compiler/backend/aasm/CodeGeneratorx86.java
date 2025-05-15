package edu.kit.kastel.vads.compiler.backend.aasm;

import edu.kit.kastel.vads.compiler.backend.regalloc.Register;
import edu.kit.kastel.vads.compiler.ir.IrGraph;
import edu.kit.kastel.vads.compiler.ir.node.*;

import java.util.*;

import static edu.kit.kastel.vads.compiler.ir.util.NodeSupport.predecessorSkipProj;

/**
 * A code generator that transforms intermediate representation (IR) graphs into x86-64 assembly code.
 * This class implements the instruction selection phase of the compiler for the x86-64 architecture,
 * handling the specific requirements and constraints of x86-64 assembly language.
 */
public class CodeGeneratorx86 {

    /**
     * Array of available x86-64 registers that can be used for general computation.
     * These include both the legacy registers (%rax through %rdi) and the extended
     * registers (%r8 through %r15) available in 64-bit mode.
     */
    private static final String[] AVAILABLE_REGISTERS = {
            "%rax", "%rbx", "%rcx", "%rdx", "%rsi", "%rdi", "%r8", "%r9", "%r12", "%r13", "%r14", "%r15"
            // Note: %r10 and %r11 are reserved for temporary operations
    };

    /**
     * Special register used for division operations in x86-64.
     * On x86-64, the quotient of a division is always stored in %rax.
     */
    private static final String DIVISION_REG = "%rax";

    /**
     * Special register used to store the remainder of division operations in x86-64.
     * On x86-64, the remainder of a division is always stored in %rdx.
     */
    private static final String REMAINDER_REG = "%rdx";

    /**
     * Maps virtual registers (from the IR) to physical x86-64 registers.
     * This mapping is created during code generation and used to translate between
     * the abstract register allocation and actual x86-64 registers.
     */
    private Map<Register, String> registerMapping = new HashMap<>();

    /**
     * Tracks the next available register index in the AVAILABLE_REGISTERS array.
     * Used during register assignment to keep track of which registers have already been allocated.
     */
    private int nextRegisterIndex = 0;

    /**
     * Generates x86-64 assembly code for a list of IR graphs representing a program.
     * Creates a complete assembly file with proper entry points, function definitions,
     * and system call handling for program termination.
     *
     * @param program A list of IR graphs to be compiled to x86-64 assembly
     * @return A string containing the generated x86-64 assembly code
     */
    public String generateCode(List<IrGraph> program) {
        StringBuilder builder = new StringBuilder();
        // Add global declarations for entry points
        // These are necessary for the linker to find the program entry points
        builder.append(".global main\n");
        builder.append(".global _main\n");
        builder.append(".text\n\n");

        // Create the main entry point, which calls _main and handles program exit
        // This setup works for both Linux and macOS
        builder.append("main:\n");
        builder.append("    call _main\n");
        builder.append("    movq %rax, %rdi\n");      // Move return value to first argument register for exit syscall
        builder.append("    movq $0x3C, %rax\n");     // Load exit syscall number into %rax
        builder.append("    syscall\n\n");            // Execute the syscall

        for (IrGraph graph : program) {
            // Process each function in the program (for L1, there's only main)
            AasmRegisterAllocator allocator = new AasmRegisterAllocator();
            Map<Node, Register> virtualRegisters = allocator.allocateRegisters(graph);

            // Reset register mapping for each function
            registerMapping.clear();
            nextRegisterIndex = 0;

            // Generate function label with an underscore prefix (required for some platforms)
            builder.append("_").append(graph.name()).append(":\n");

            // Function prologue - sets up the stack frame
            // Calculate required stack space for spilled registers
            int stackSpaceNeeded = calculateStackSpaceNeeded(virtualRegisters.size());

            // Adjust the function prologue to allocate stack space
            builder.append("    pushq %rbp\n");
            builder.append("    movq %rsp, %rbp\n");
            if (stackSpaceNeeded > 0) {
                builder.append("    subq $").append(stackSpaceNeeded).append(", %rsp\n");
            }

            // Generate the function body
            generateForGraphX86(graph, builder, virtualRegisters);

            // Note: The function epilogue is handled by the ReturnNode
        }
        return builder.toString();
    }

    private int calculateStackSpaceNeeded(int virtualRegCount) {
        int physicalRegCount = AVAILABLE_REGISTERS.length;
        int spilledRegCount = Math.max(0, virtualRegCount - physicalRegCount);

        // Each register needs 8 bytes, and ensure 16-byte alignment for the stack
        int bytes = spilledRegCount * 8;
        return (bytes + 15) & ~15;  // Round up to multiple of 16
    }

    /**
     * Generates x86-64 assembly code for a single IR graph (representing a function).
     * The generation starts from the end block and works backwards through the graph.
     *
     * @param graph The IR graph to generate code for
     * @param builder The StringBuilder to append generated code to
     * @param virtualRegisters Map of IR nodes to their allocated virtual registers
     */
    private void generateForGraphX86(IrGraph graph, StringBuilder builder, Map<Node, Register> virtualRegisters) {
        Set<Node> visited = new HashSet<>();
        // Start traversal from the end block to ensure correct execution order
        scan(graph.endBlock(), visited, builder, virtualRegisters);
    }

    /**
     * Recursively scans the IR graph starting from a given node, generating code
     * for each node after its predecessors have been processed. This ensures correct
     * execution order in the generated code despite using a backwards traversal.
     *
     * @param node The current node being processed
     * @param visited Set of nodes that have already been visited
     * @param builder The StringBuilder to append generated code to
     * @param registers Map of IR nodes to their allocated registers
     */
    protected void scan(Node node, Set<Node> visited, StringBuilder builder, Map<Node, Register> registers) {
        // First process all predecessors not yet visited
        for (Node predecessor : node.predecessors()) {
            // The add() method returns true if the element was added (wasn't already in the set)
            if (visited.add(predecessor)) {
                scan(predecessor, visited, builder, registers);
            }
        }

        // Generate code for the current node based on its type
        switch (node) {
            case AddNode add -> binaryX86(builder, registers, add, "add");
            case SubNode sub -> binaryX86(builder, registers, sub, "sub");
            case MulNode mul -> binaryX86(builder, registers, mul, "mul");
            case DivNode div -> binaryX86(builder, registers, div, "div");
            case ModNode mod -> binaryX86(builder, registers, mod, "mod");
            case ReturnNode r -> {
                // Get the return value
                String resultReg = registers.get(predecessorSkipProj(r, ReturnNode.RESULT)).toString();

                // Move result to %rax (return value register)
                if (!resultReg.equals("%rax")) {
                    builder.append("    movq ").append(resultReg).append(", %rax\n");
                }

                // Function epilogue - restore stack pointer if needed
                if (currentStackOffset > 16) {
                    builder.append("    movq %rbp, %rsp\n");
                }

                // Restore base pointer and return
                builder.append("    popq %rbp\n");
                builder.append("    ret");
            }
            case ConstIntNode c -> builder.repeat(" ", 2)
                    .append(registers.get(c))
                    .append(" = const ")
                    .append(c.value());
            case Phi _ -> throw new UnsupportedOperationException("phi");
            case Block _, ProjNode _, StartNode _ -> {
                // These node types don't generate assembly instructions
                // Skip adding a newline
                return;
            }
        }
        builder.append("\n");
    }

    /**
     * Generates x86-64 assembly code for binary operations (add, sub, mul).
     * Handles the two-address nature of x86-64 instructions by ensuring the
     * target register contains the left operand before performing the operation.
     *
     * @param builder The StringBuilder to append generated code to
     * @param virtualRegisters Map of IR nodes to their allocated virtual registers
     * @param node The binary operation node being processed
     * @param opcode The x86-64 operation code to use (add, sub, etc.)
     */
    private void binaryX86(StringBuilder builder,
                           Map<Node, Register> virtualRegisters,
                           BinaryOperationNode node,
                           String opcode
    ) {
        Register targetReg = virtualRegisters.get(node);
        Register leftReg = virtualRegisters.get(predecessorSkipProj(node, BinaryOperationNode.LEFT));
        Register rightReg = virtualRegisters.get(predecessorSkipProj(node, BinaryOperationNode.RIGHT));

        String x86TargetReg = getX86Register(targetReg);
        String x86LeftReg = getX86Register(leftReg);
        String x86RightReg = getX86Register(rightReg);

        // Check if target is a memory location
        boolean targetIsMemory = x86TargetReg.contains("(");

        // If target is memory location, we need an intermediate register
        if (targetIsMemory) {
            // Use a temporary register (e.g., %r11 which we'll reserve for this purpose)
            String tempReg = "%r11";

            // Load left operand into temp register
            builder.append("    movq ").append(x86LeftReg).append(", ").append(tempReg).append("\n");

            // Perform operation with right operand
            builder.append("    ").append(opcode).append(" ").append(x86RightReg).append(", ").append(tempReg).append("\n");

            // Store result back to memory
            builder.append("    movq ").append(tempReg).append(", ").append(x86TargetReg);
        } else {
            // Target is a register - proceed as before
            if (!x86TargetReg.equals(x86LeftReg)) {
                // If left operand is in memory, use movq to load it
                builder.append("    movq ").append(x86LeftReg).append(", ").append(x86TargetReg).append("\n");
            }

            // If right operand is in memory and operation requires a register, handle specially
            if (x86RightReg.contains("(") && (opcode.equals("imul") || opcode.equals("idiv"))) {
                // For operations that can't take a memory operand directly
                String tempReg = "%r10";  // Use a reserved temporary register
                builder.append("    movq ").append(x86RightReg).append(", ").append(tempReg).append("\n");
                builder.append("    ").append(opcode).append(" ").append(tempReg).append(", ").append(x86TargetReg);
            } else {
                // For operations that can take a memory operand
                builder.append("    ").append(opcode).append(" ").append(x86RightReg).append(", ").append(x86TargetReg);
            }
        }
    }



    /**
     * Maps a virtual register to a physical x86-64 register or a stack location.
     * If all physical registers are in use, some values will be "spilled" to memory.
     *
     * @param virtualReg The virtual register to map
     * @return The corresponding x86-64 register name or a stack reference
     */
    private String getX86Register(Register virtualReg) {
        if (!registerMapping.containsKey(virtualReg)) {
            if (nextRegisterIndex < AVAILABLE_REGISTERS.length) {
                // Assign a physical register if available
                String x86Reg = AVAILABLE_REGISTERS[nextRegisterIndex++];
                registerMapping.put(virtualReg, x86Reg);
            } else {
                // Implement register spilling to stack
                // Allocate a new stack slot (relative to %rbp)
                int stackOffset = allocateStackSlot();
                String stackLocation = String.format("%d(%%rbp)", -stackOffset);
                registerMapping.put(virtualReg, stackLocation);
            }
        }
        return registerMapping.get(virtualReg);
    }

    // Track stack space needed for register spilling
    private int currentStackOffset = 16; // Start after saved %rbp and return address

    /**
     * Allocates a new stack slot for register spilling.
     * Each slot is 8 bytes (64 bits) for our x86-64 implementation.
     *
     *
     * @return The byte offset from %rbp for this stack slot
     */
    private int allocateStackSlot() {
        // Ensure stack alignment (8 bytes for 64-bit values)
        currentStackOffset = (currentStackOffset + 7) & ~7;
        int offset = currentStackOffset;
        currentStackOffset += 8; // Allocate 8 bytes
        return offset;
    }


}
