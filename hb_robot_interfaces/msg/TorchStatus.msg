#TorchStatus.msg
builtin_interfaces/Time stamp
string segment_name

bool io_connected
bool command_on
bool output_on
bool transfer_detected

bool analog_valid
float64 analog_input_value
uint8 analog_domain

bool arc_voltage_valid
float64 arc_voltage_v

bool ready
bool fault
string fault_reason
