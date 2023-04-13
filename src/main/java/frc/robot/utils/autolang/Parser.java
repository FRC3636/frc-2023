package frc.robot.utils.autolang;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utils.autolang.Lexer.GamePieceVariant;
import frc.robot.utils.autolang.Lexer.PathingModeVariant;

/**
 * Uses tokens from a Lexer to create a machine-readable representation of the
 * programming language and report syntax errors.
 */
public class Parser {
    private final Lexer lexer;

    public Parser(Lexer lexer) {
        this.lexer = lexer;
    }

    public class AutoLangSyntaxError extends Exception {
        public AutoLangSyntaxError(String message) {
            super(message);
        }
    }

    public class AutoLangProgram {
        public final CommandAST[] commands;

        public AutoLangProgram(CommandAST[] commands) {
            this.commands = commands;
        }

        public Command compile() {
            SequentialCommandGroup output = new SequentialCommandGroup();

            for (CommandAST statement : commands) {
                Command command = statement.compile();
                output.addCommands(command);
            }

            return output;
        }
    }

    public abstract class CommandAST {
        public abstract Command compile();
    }

    public class IntakeCommandAST extends CommandAST {
        public GamePieceVariant gamePiece;
        public int index;
        public PathingModeVariant pathingMode;

        public
    }

    public class GamePieceAST {
    }

    public CommandAST parseStatement() {
        Token token
    }
}
