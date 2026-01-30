# JSON-RPC error message

int16 code       # Error code
string message   # Error message
string id        # Request ID

# Standard JSON-RPC error codes
int16 ERROR_INVALID_JSON = -32700       # Invalid JSON received
int16 ERROR_INVALID_REQUEST = -32600    # Not a valid Request object
int16 ERROR_METHOD_NOT_FOUND = -32601   # Method does not exist
int16 ERROR_INVALID_PARAMS = -32602     # Invalid method parameters
int16 ERROR_INTERNAL = -32603           # Internal JSON-RPC error
int16 ERROR_SERVER_FIRST = -32000       # Reserved for server errors (start)
int16 ERROR_SERVER_LAST = -32099        # Reserved for server errors (end)
