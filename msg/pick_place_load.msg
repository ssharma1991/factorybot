uint8 GO_PICK=0     # An unloaded vehicle navigates to a goal
uint8 PICK=1        # An unloaded vehicle picks up the load
uint8 GO_PLACE=2    # A loaded vehicle navigates to a goal
uint8 PLACE=3       # A loaded vehicle drops its payload

int32 action
float64 x
float64 y
float64 theta