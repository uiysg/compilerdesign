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
            "%rax", "%rbx", "%rcx", "%rdx", "%rsi", "%rdi", "%r8", "%r9", "%r10", "%r11", "%r12", "%r13", "%r14", "%r15"
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
            builder.append("    pushq %rbp\n");       // Save the old base pointer
            builder.append("    movq %rsp, %rbp\n");  // Set new base pointer to current stack pointer

            // Generate the function body
            generateForGraphX86(graph, builder, virtualRegisters);

            // Note: The function epilogue is handled by the ReturnNode
        }
        return builder.toString();
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
            case ReturnNode r -> builder.repeat(" ", 2).append("ret ")
                    .append(registers.get(predecessorSkipProj(r, ReturnNode.RESULT)));
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
        // Get the virtual registers assigned to this operation
        Register targetReg = virtualRegisters.get(node);
        Register leftReg = virtualRegisters.get(predecessorSkipProj(node, BinaryOperationNode.LEFT));
        Register rightReg = virtualRegisters.get(predecessorSkipProj(node, BinaryOperationNode.RIGHT));

        // Map virtual registers to actual x86-64 registers
        String x86TargetReg = getX86Register(targetReg);
        String x86LeftReg = getX86Register(leftReg);
        String x86RightReg = getX86Register(rightReg);

        // x86-64 uses two-address instructions (dest is also a source)
        // If the target register is different from the left operand register,
        // we need to copy the left operand to the target register first
        if (!x86TargetReg.equals(x86LeftReg)) {
            builder.append("    movq ").append(x86LeftReg).append(", ").append(x86TargetReg).append("\n");
        }

        // Now perform the operation with the right operand
        // Format: opcode src, dest (where dest is also used as the first operand)
        builder.append("    ").append(opcode).append(" ").append(x86RightReg).append(", ").append(x86TargetReg);
    }

    /**
     * Generates x86-64 assembly code for division or modulo operations.
     * Handles the special requirements of x86-64 division, which uses fixed
     * registers (%rax for dividend and quotient, %rdx for remainder).
     *
     * @param builder The StringBuilder to append generated code to
     * @param virtualRegisters Map of IR nodes to their allocated virtual registers
     * @param node The division or modulo operation node being processed
     * @param isDiv True if this is a division operation, false if it's a modulo operation
     */
    private void divisionX86(
            StringBuilder builder,
            Map<Node, Register> virtualRegisters,
            BinaryOperationNode node,
            boolean isDiv // false for mod
    ) {
        // Get the virtual registers assigned to this operation
        Register targetReg = virtualRegisters.get(node);
        Register leftReg = virtualRegisters.get(predecessorSkipProj(node, BinaryOperationNode.LEFT));
        Register rightReg = virtualRegisters.get(predecessorSkipProj(node, BinaryOperationNode.RIGHT));

        // Map virtual registers to actual x86-64 registers
        String x86TargetReg = getX86Register(targetReg);
        String x86LeftReg = getX86Register(leftReg);
        String x86RightReg = getX86Register(rightReg);

        // Division on x86-64 requires specific register usage:
        // - Dividend must be in %rax
        // - Division result (quotient) is placed in %rax
        // - Remainder is placed in %rdx

        // Move the dividend (left operand) to %rax
        builder.append("    movq ").append(x86LeftReg).append(", %rax\n");

        // Sign-extend %rax into %rdx (required before signed division)
        builder.append("    cqto\n");

        // Perform the division operation
        builder.append("    idivq ").append(x86RightReg).append("\n");

        // Determine which register contains our result (quotient or remainder)
        String resultReg = isDiv ? "%rax" : "%rdx";

        // If the target register is not already the result register,
        // copy the result to the target register
        if (!x86TargetReg.equals(resultReg)) {
            builder.append("    movq ").append(resultReg).append(", ").append(x86TargetReg);
        }
    }

    /**
     * Maps a virtual register to a physical x86-64 register.
     * If the virtual register hasn't been mapped yet, assigns the next
     * available register from AVAILABLE_REGISTERS.
     *
     * @param virtualReg The virtual register to map
     * @return The corresponding x86-64 register name
     * @throws UnsupportedOperationException if all available registers are used
     *         and register spilling to memory would be required
     */
    private String getX86Register(Register virtualReg) {
        if (!registerMapping.containsKey(virtualReg)) {
            // If this virtual register hasn't been assigned an x86 register yet, assign one
            if (nextRegisterIndex < AVAILABLE_REGISTERS.length) {
                // Get the next available register and increment the index
                String x86Reg = AVAILABLE_REGISTERS[nextRegisterIndex++];
                registerMapping.put(virtualReg, x86Reg);
            } else {
                // We've run out of registers - in a real implementation, we would
                // spill some registers to memory (stack) at this point
                throw new UnsupportedOperationException("Register spilling not implemented, too many registers needed");
            }
        }
        return registerMapping.get(virtualReg);
    }


}
