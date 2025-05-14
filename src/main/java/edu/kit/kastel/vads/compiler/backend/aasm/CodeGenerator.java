package edu.kit.kastel.vads.compiler.backend.aasm;

import edu.kit.kastel.vads.compiler.backend.regalloc.Register;
import edu.kit.kastel.vads.compiler.ir.IrGraph;
import edu.kit.kastel.vads.compiler.ir.node.AddNode;
import edu.kit.kastel.vads.compiler.ir.node.BinaryOperationNode;
import edu.kit.kastel.vads.compiler.ir.node.Block;
import edu.kit.kastel.vads.compiler.ir.node.ConstIntNode;
import edu.kit.kastel.vads.compiler.ir.node.DivNode;
import edu.kit.kastel.vads.compiler.ir.node.ModNode;
import edu.kit.kastel.vads.compiler.ir.node.MulNode;
import edu.kit.kastel.vads.compiler.ir.node.Node;
import edu.kit.kastel.vads.compiler.ir.node.Phi;
import edu.kit.kastel.vads.compiler.ir.node.ProjNode;
import edu.kit.kastel.vads.compiler.ir.node.ReturnNode;
import edu.kit.kastel.vads.compiler.ir.node.StartNode;
import edu.kit.kastel.vads.compiler.ir.node.SubNode;

import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static edu.kit.kastel.vads.compiler.ir.util.NodeSupport.predecessorSkipProj;

/**
 * Transforms intermediate representation (IR) graphs into abstract assembly code.
 * This class implements the instruction selection phase of the compiler,
 * mapping IR nodes to corresponding abstract assembly instructions.
 */
public class CodeGenerator {

    /**
     * Generates abstract assembly code for a list of IR graphs representing a program.
     *
     * @param program A list of IR graphs to be compiled
     * @return A string containing the generated abstract assembly code
     */
    public String generateCode(List<IrGraph> program) {
        StringBuilder builder = new StringBuilder();
        for (IrGraph graph : program) {
            // Create a register allocator for each function
            AasmRegisterAllocator allocator = new AasmRegisterAllocator();

            // Allocate registers for nodes in the graph
            Map<Node, Register> registers = allocator.allocateRegisters(graph);

            // Generate function header
            builder.append("function ")
                    .append(graph.name())
                    .append(" {\n");

            // Generate code for the function body
            generateForGraph(graph, builder, registers);

            // Close the function definition
            builder.append("}");
        }
        return builder.toString();
    }

    /**
     * Generates code for a single IR graph (representing a function).
     * The generation starts from the end block and works backwards through the graph.
     *
     * @param graph The IR graph to generate code for
     * @param builder The StringBuilder to append generated code to
     * @param registers Map of IR nodes to their allocated registers
     */
    private void generateForGraph(IrGraph graph, StringBuilder builder, Map<Node, Register> registers) {
        Set<Node> visited = new HashSet<>();
        // Start traversal from the end block
        scan(graph.endBlock(), visited, builder, registers);
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
    private void scan(Node node, Set<Node> visited, StringBuilder builder, Map<Node, Register> registers) {
        // First process all predecessors not yet visited
        for (Node predecessor : node.predecessors()) {
            // The add() method returns true if the element was added (i.e., wasn't already in the set)
            if (visited.add(predecessor)) {
                scan(predecessor, visited, builder, registers);
            }
        }

        // Generate code for the current node using pattern matching
        switch (node) {
            // Handle arithmetic operations using the binary helper method
            case AddNode add -> binary(builder, registers, add, "add");
            case SubNode sub -> binary(builder, registers, sub, "sub");
            case MulNode mul -> binary(builder, registers, mul, "mul");
            case DivNode div -> binary(builder, registers, div, "div");
            case ModNode mod -> binary(builder, registers, mod, "mod");

            // Generate return instruction with the result value
            case ReturnNode r -> builder.repeat(" ", 2).append("ret ")
                    .append(registers.get(predecessorSkipProj(r, ReturnNode.RESULT)));

            // Generate constant loading instruction
            case ConstIntNode c -> builder.repeat(" ", 2)
                    .append(registers.get(c))
                    .append(" = const ")
                    .append(c.value());

            // Phi nodes are not supported in this implementation
            case Phi _ -> throw new UnsupportedOperationException("phi");

            // Skip code generation for these node types as they don't need assembly instructions
            case Block _, ProjNode _, StartNode _ -> {
                // do nothing, skip line break
                return;
            }
        }
        // Add a newline after each instruction (except for skipped nodes)
        builder.append("\n");
    }

    /**
     * Helper method to generate code for binary operations (add, sub, mul, div, mod).
     * Creates a three-address instruction in the form: dest = op left right
     *
     * @param builder The StringBuilder to append generated code to
     * @param registers Map of IR nodes to their allocated registers
     * @param node The binary operation node being processed
     * @param opcode The operation code to use in the assembly instruction
     */
    private static void binary(
            StringBuilder builder,
            Map<Node, Register> registers,
            BinaryOperationNode node,
            String opcode
    ) {
        // Indent the instruction with 2 spaces
        builder.repeat(" ", 2)
                // Destination register
                .append(registers.get(node))
                .append(" = ")
                // Operation code (add, sub, mul, div, mod)
                .append(opcode)
                .append(" ")
                // Left operand register (skipping projection nodes)
                .append(registers.get(predecessorSkipProj(node, BinaryOperationNode.LEFT)))
                .append(" ")
                // Right operand register (skipping projection nodes)
                .append(registers.get(predecessorSkipProj(node, BinaryOperationNode.RIGHT)));
    }
}