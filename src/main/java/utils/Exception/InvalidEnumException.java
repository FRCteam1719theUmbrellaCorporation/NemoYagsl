package utils.Exception;

public class InvalidEnumException extends Exception {
    public InvalidEnumException() {
        super();
    }

    public InvalidEnumException(String message) {
        super(message);
    }

    public InvalidEnumException(String message, Throwable cause) {
        super(message, cause);
    }

    public InvalidEnumException(Throwable cause) {
        super(cause);
    }
}
