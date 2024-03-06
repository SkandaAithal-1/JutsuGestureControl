## What is it?

- This custom header file is the first iteration of our custom commands
- It `#include`s the `gnc_functions.hpp` header file and adds on to it
- It has commands related to `cmd_vel` which were absent in the `gnc_functions.hpp` header

## How to use?

- The `header_test.cpp` file is an example which shows how you can use this header
- Call the moving functions (forward, up, rotate) while passing `velocity` and `duration` as parameters
- There are default values for each, so calling function as is, is valid
- There is a `stopped` variable, which needs to be `false` for normal functioning
- In your node, create a publisher which sends start/stop signal to the main file, or set it statically to false
