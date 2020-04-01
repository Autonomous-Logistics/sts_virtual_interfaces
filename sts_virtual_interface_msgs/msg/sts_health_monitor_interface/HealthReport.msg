
### Meta information
###
time stamp
string uuid
string sender

### User defined messages
###
string machine_message
string human_message


### Severity of the problem ranging from 0(Ignore) to 5(Highest)
### 	SAFE = 0x00, 
###		NON_CRITICAL_INFO = 0x01, 
###		NON_CRITICAL_WARNING = 0x02, WARNING = 0x03, 
###		ERROR = 0x04, CRITICAL_ERROR = 0x05
###
uint32 severity
string severity_str


### Report/Error-Type defining the direction/nature and purpose/specifics of the problem
### Most common examples: 
### 	NONE = 0x00, OTHER = 0x01, UNKNOWN = 0x02,
### 	COMMUNICATION_ERROR = 0x10, 
### 	ACCURACY_ERROR = 0x20,
### 	RANGE_ERROR = 0x30,
###		RANGE_MISMATCH = 0x31,
###		INPUT_ERROR = 0x40,
###		OUTPUT_ERROR = 0x50,
###		USER_ERROR = 0x60,
###		OPERATION_RANGE_ERROR = 0xE0,
###		DANGER = 0xF0,
###
uint32 type
string type_str


### The [participant] type of the source of the problem (who is probably responsible?!)
###	AND
### The [participant] type of the target of the problem (who is the likely victim?!)
### Participant types are defined like this:
### 	NONE = 0x00, OTHER = 0x01, UNKNOWN = 0x02,
### 	TOPIC = 0x10,
###		NODE = 0x40,
###		GROUP = 0x70,
###		SYSTEM = 0xF0
###
uint32 source_type
string source_type_str
string source
uint32 target_type
string target_type_str
string target