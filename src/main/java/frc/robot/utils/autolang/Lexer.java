package frc.robot.utils.autolang;

import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.Optional;

public class Lexer {
    private Iterator<Character> remainingChars;
    private Optional<Character> lastChar;
    private SourceLocation sourceLoc;

    public Lexer(Iterator<Character> chars) {
        this.remainingChars = chars;
        this.sourceLoc = new SourceLocation(0, 0);
        this.nextChar();
    }

    public class SourceLocation {
        public int startIndex;
        public int endIndex;

        public SourceLocation(int startIndex, int endIndex) {
            this.startIndex = startIndex;
            this.endIndex = endIndex;
        }
    }

    public abstract class Token {
    }

    public class IdentifierToken extends Token {
        public final String value;

        public IdentifierToken(String value) {
            this.value = value;
        }
    }

    public class CommandToken extends Token {
        public final CommandVariant value;

        public CommandToken(CommandVariant value) {
            this.value = value;
        }
    }

    public enum CommandVariant {
        Intake,
        Score,
        Balance,
        LeaveCommunity,
        Wait,
        Drop;
    }

    public class GamePieceToken extends Token {
        public final GamePieceVariant value;

        public GamePieceToken(GamePieceVariant value) {
            this.value = value;
        }
    }

    public enum GamePieceVariant {
        Cone,
        Cube;
    }

    public class ClosestGridToken extends Token {
    }

    public class NodeLevelToken extends Token {
        public final NodeLevelVariant value;

        public NodeLevelToken(NodeLevelVariant value) {
            this.value = value;
        }
    }

    public enum NodeLevelVariant {
        Low,
        Mid,
        High;
    }

    public class NodeColumnToken extends Token {
        public final NodeColumnVariant value;

        public NodeColumnToken(NodeColumnVariant value) {
            this.value = value;
        }
    }

    public enum NodeColumnVariant {
        ConeLeft,
        Cube,
        ConeRight;
    }

    public class PathingModeToken extends Token {
        public final PathingModeVariant value;

        public PathingModeToken(PathingModeVariant value) {
            this.value = value;
        }
    }

    public enum PathingModeVariant {
        AvoidObstacles,
        IgnoreObstacles;
    }

    public class NumberToken extends Token {
        public final Double value;

        public NumberToken(Double value) {
            this.value = value;
        }
    }

    public class UnknownToken extends Token {
        public final char value;

        public UnknownToken(char value) {
            this.value = value;
        }
    }

    private void nextChar() {
        try {
            var next = this.remainingChars.next();
            this.sourceLoc.endIndex += 1;
            this.lastChar = Optional.of(next);
        } catch (NoSuchElementException _e) {
            this.lastChar = Optional.empty();
        }
    }

    private char unwrapLastChar() {
        return this.lastChar.orElseThrow();
    }

    private void skipWhitespace() {
        while (this.lastChar.isPresent()) {
            var lastChar = this.lastChar.get();
            if (!Character.isWhitespace(lastChar)) {
                break;
            }
            this.nextChar();
        }
    }

    /// Find an identifier (or keyword like `def`).
    /// Identifiers start with `a-z`/`A-Z` and can contain `a-z`/`A-Z`/`0-9`.
    /// Will panic if this is the end of the file.
    /// Returns `None` if this isn't an identifier.
    private Optional<String> getIdentifier() {
        var lastChar = this.unwrapLastChar();
        if (!Character.isAlphabetic(lastChar)) {
            return Optional.empty();
        }

        var identString = Character.toString(lastChar);

        while (true) {
            this.nextChar();
            if (this.lastChar.isPresent()) {
                lastChar = this.lastChar.get();
                if (Character.isAlphabetic(lastChar)) {
                    identString += lastChar;
                    continue;
                }
            }
            break;
        }

        return Optional.of(identString);
    }

    /// Find a number/float, like `12.34`.
    /// Returns `None` if this isn't a number.
    private Optional<Double> getNumber() {
        var lastChar = this.unwrapLastChar();
        if (!Character.isDigit(lastChar)) {
            return Optional.empty();
        }

        var numString = Character.toString(lastChar);
        boolean allowDot = true;

        while (true) {
            this.nextChar();
            if (this.lastChar.isPresent()) {
                lastChar = this.lastChar.get();
                if (Character.isDigit(lastChar) || (allowDot && lastChar == '.')) {
                    if (lastChar == '.') {
                        allowDot = false;
                    }

                    numString += lastChar;
                    continue;
                }
            }
            break;
        }

        Double value = Double.parseDouble(numString); // this should never fail
        return Optional.of(value);
    }

    /// Move the cursor to the end of the line if there is a comment.
    /// Returns whether there was a coment.
    private boolean readComment() {
        if (this.unwrapLastChar() != '#') {
            return false;
        }

        while (true) {
            this.nextChar();
            if (this.lastChar.isPresent()) {
                if (this.lastChar.get() == '\n') {
                    break;
                }
            } else {
                // end of file, prevent an infinite read
                break;
            }
        }

        return true;
    }

    private Token identifierToToken(String ident) {
        switch (ident) {
            case "intake":
                return new CommandToken(CommandVariant.Intake);
            case "score":
                return new CommandToken(CommandVariant.Score);
            case "balance":
                return new CommandToken(CommandVariant.Balance);
            case "leave_community":
                return new CommandToken(CommandVariant.LeaveCommunity);
            case "wait":
                return new CommandToken(CommandVariant.Wait);
            case "drop":
                return new CommandToken(CommandVariant.Drop);
            case "closest_grid":
                return new ClosestGridToken();
            case "low":
                return new NodeLevelToken(NodeLevelVariant.Low);
            case "mid":
                return new NodeLevelToken(NodeLevelVariant.Mid);
            case "high":
                return new NodeLevelToken(NodeLevelVariant.High);
            case "cone_left":
                return new NodeColumnToken(NodeColumnVariant.ConeLeft);
            case "cube":
                return new NodeColumnToken(NodeColumnVariant.Cube);
            case "cone_right":
                return new NodeColumnToken(NodeColumnVariant.ConeRight);
            case "avoid_obstacles":
                return new PathingModeToken(PathingModeVariant.AvoidObstacles);
            case "ignore_obstacles":
                return new PathingModeToken(PathingModeVariant.IgnoreObstacles);
            default:
                return new IdentifierToken(ident);
        }
    }

    /// Read the next token and return the variant, or `None` if the end of the file
    /// has been reached.
    public Optional<Token> getToken() {
        this.skipWhitespace();
        if (this.lastChar.isEmpty()) {
            return Optional.empty();
        }

        var ident = this.getIdentifier();
        if (ident.isPresent()) {
            var identString = ident.get();
            return Optional.of(identifierToToken(identString));
        }

        var num = this.getNumber();
        if (num.isPresent()) {
            return Optional.of(new NumberToken(num.get()));
        }

        if (this.readComment()) {
            return this.getToken();
        }

        var token = new UnknownToken(this.unwrapLastChar());
        // move to the next character so we dont re-parse this one
        this.nextChar();
        return Optional.of(token);
    }
}
