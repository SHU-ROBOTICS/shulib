# Build Errors

**When your code won't compile**

---

## Quick Fixes

### "It was working, now it's not"

```bash
# Clean and rebuild
pros make clean
pros make
```

### "Weird errors after git pull"

```bash
# Full clean
rm -rf bin/
pros make clean
pros make
```

### "Nothing makes sense"

```bash
# Check you're in the right directory
pwd
# Should be in shulib project folder

# Check PROS is working
pros --version
```

---

## Common Errors

### fatal error: 'xxx.hpp' file not found

```
fatal error: 'shulib/core/chassis.hpp' file not found
#include "shulib/core/chassis.hpp"
         ^~~~~~~~~~~~~~~~~~~~~~~~~~
```

**Cause:** Include path is wrong or file doesn't exist.

**Fixes:**

1. **Check the file exists:**
   ```bash
   ls include/shulib/core/chassis.hpp
   ```

2. **Check spelling and path:**
   ```cpp
   // Wrong
   #include "shulib/Core/chassis.hpp"  // Capital C!
   #include "shulib/chassis.hpp"       // Missing /core/
   
   // Right
   #include "shulib/core/chassis.hpp"
   ```

3. **Check from project root:**
   All includes should be relative to `include/` folder.

---

### undefined reference to 'xxx'

```
undefined reference to `shulib::Chassis::drive(float, float, float)'
```

**Cause:** Function is declared but not implemented, or .cpp file isn't being compiled.

**Fixes:**

1. **Check implementation exists:**
   ```bash
   # Search for the function
   grep -r "Chassis::drive" src/
   ```

2. **Check file is being compiled:**
   Make sure the .cpp file is in `src/` directory (not accidentally in `include/`).

3. **Check for typos:**
   ```cpp
   // Declaration (in .hpp)
   void drive(float x, float y, float z);
   
   // Implementation (in .cpp) - MUST MATCH EXACTLY
   void Chassis::drive(float x, float y, float z) {
       // ...
   }
   ```

4. **Check namespace:**
   ```cpp
   // If declared in namespace:
   namespace shulib {
       class Chassis {
           void drive(float x, float y, float z);
       };
   }
   
   // Implementation must use namespace:
   void shulib::Chassis::drive(float x, float y, float z) {
       // ...
   }
   ```

---

### No robot selected

```
#error "No robot selected! Define ROBOT_XEBEC or ROBOT_QUEENS_REVENGE in config.hpp"
```

**Cause:** No robot is uncommented in config.hpp.

**Fix:**

Edit `config.hpp`:
```cpp
// Uncomment ONE robot:
#define ROBOT_XEBEC              // ← Uncomment this line
// #define ROBOT_QUEENS_REVENGE
```

---

### redefinition of 'xxx'

```
error: redefinition of 'void foo()'
```

**Cause:** Function or variable defined multiple times.

**Fixes:**

1. **Check for duplicate includes:**
   Add include guards:
   ```cpp
   #pragma once  // Add this at top of every .hpp file
   ```

2. **Check for duplicate definitions:**
   Only DECLARE in .hpp, DEFINE in .cpp:
   ```cpp
   // In .hpp (declaration only)
   void foo();
   
   // In .cpp (definition)
   void foo() {
       // implementation
   }
   ```

3. **For global variables:**
   ```cpp
   // In .hpp
   extern int myGlobal;  // Declaration
   
   // In ONE .cpp file
   int myGlobal = 0;     // Definition
   ```

---

### expected ';' / expected '}'

```
error: expected ';' after class definition
error: expected '}' at end of input
```

**Cause:** Missing semicolon or brace somewhere.

**Fix:**

Check the line BEFORE the error:
```cpp
class MyClass {
    // ...
}   // ← Missing semicolon here!

// Should be:
class MyClass {
    // ...
};  // ← Semicolon after class
```

Also check for:
- Missing `}` to close a function
- Missing `}` to close an if/while/for
- Missing `)` in function call

---

### 'xxx' does not name a type

```
error: 'Pose' does not name a type
```

**Cause:** Type is not defined or not included.

**Fixes:**

1. **Add the include:**
   ```cpp
   #include "shulib/core/pose.hpp"  // Include the type's header
   ```

2. **Check namespace:**
   ```cpp
   // If Pose is in shulib namespace:
   shulib::Pose myPose;  // Use full name
   
   // Or add using statement:
   using shulib::Pose;
   Pose myPose;
   ```

3. **Check include order:**
   Some headers depend on others. Make sure base types are included first.

---

### 'xxx' is not a member of 'yyy'

```
error: 'setIntake' is not a member of 'Mechanisms'
```

**Cause:** Method doesn't exist on that class.

**Fixes:**

1. **Check spelling:**
   ```cpp
   mech.setintake(127);  // Wrong - lowercase 'i'
   mech.setIntake(127);  // Right
   ```

2. **Check it's declared:**
   Look in the .hpp file:
   ```cpp
   class Mechanisms {
   public:
       void setIntake(int power);  // Is it here?
   };
   ```

3. **Check you're calling on right object:**
   ```cpp
   chassis.setIntake(127);  // Wrong - Chassis doesn't have setIntake
   mech.setIntake(127);     // Right - Mechanisms has it
   ```

---

### cannot convert 'xxx' to 'yyy'

```
error: cannot convert 'std::string' to 'int'
```

**Cause:** Type mismatch in assignment or function call.

**Fixes:**

1. **Check parameter types:**
   ```cpp
   // Function expects int
   void setMotor(int port);
   
   // You're passing string
   setMotor("12");  // Wrong
   setMotor(12);    // Right
   ```

2. **Check return type:**
   ```cpp
   // Function returns Pose
   Pose getPose();
   
   int x = chassis.getPose();  // Wrong - can't assign Pose to int
   Pose p = chassis.getPose(); // Right
   int x = chassis.getPose().x; // Also right
   ```

---

### use of undeclared identifier 'xxx'

```
error: use of undeclared identifier 'chassis'
```

**Cause:** Variable hasn't been declared in this scope.

**Fixes:**

1. **Declare the variable:**
   ```cpp
   chassis.drive(0, 0, 0);  // Error - chassis not declared
   
   Chassis chassis;         // Declare it first
   chassis.drive(0, 0, 0);  // Now it works
   ```

2. **Check scope:**
   ```cpp
   if (true) {
       Chassis chassis;
   }
   chassis.drive(0, 0, 0);  // Error - chassis not in scope
   
   // Move declaration outside:
   Chassis chassis;
   if (true) {
       // use chassis
   }
   chassis.drive(0, 0, 0);  // Works
   ```

3. **Check parameter name:**
   ```cpp
   void run(Chassis& c) {
       chassis.drive(0, 0, 0);  // Wrong - parameter is 'c'
       c.drive(0, 0, 0);        // Right
   }
   ```

---

### Too few/many arguments to function

```
error: too few arguments to function 'void drive(float, float, float)'
```

**Cause:** Function call has wrong number of arguments.

**Fix:**

Check function signature:
```cpp
// Function expects 3 arguments
void drive(float x, float y, float z);

chassis.drive(0, 50);        // Error - only 2 arguments
chassis.drive(0, 50, 0);     // Right - 3 arguments
```

---

## Toolchain Errors

### pros: command not found

```
bash: pros: command not found
```

**Fix:**

1. Check PROS CLI is installed:
   ```bash
   pip install pros-cli --break-system-packages
   ```

2. Check PATH:
   ```bash
   export PATH="$HOME/.local/bin:$PATH"
   ```

3. Add to ~/.bashrc permanently:
   ```bash
   echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
   source ~/.bashrc
   ```

---

### arm-none-eabi-g++: command not found

```
make: arm-none-eabi-g++: Command not found
```

**Fix:**

Install ARM toolchain:
```bash
sudo apt install gcc-arm-none-eabi
```

---

### Permission denied

```
make: bin/output.elf: Permission denied
```

**Fix:**

```bash
rm -rf bin/
pros make
```

---

## Linker Errors

### Multiple definition of 'xxx'

```
multiple definition of `myFunction()'
```

**Cause:** Function defined in header and included multiple times.

**Fix:**

Move implementation to .cpp file, or use `inline`:

```cpp
// In .hpp - use inline for header-only functions
inline void myFunction() {
    // ...
}

// OR move to .cpp file
```

---

### Undefined reference to 'main'

```
undefined reference to `main'
```

**Cause:** main.cpp has issues or isn't being compiled.

**Fix:**

1. Check main.cpp exists and has proper PROS entry points:
   ```cpp
   void initialize() { }
   void disabled() { }
   void competition_initialize() { }
   void autonomous() { }
   void opcontrol() { }
   ```

2. Check it's in `src/` directory.

---

## Debugging Build Errors

### Read the FIRST error

Compilers often cascade errors. Fix the first one, rebuild, repeat.

```
error: 'foo' not declared        ← FIX THIS ONE FIRST
error: expected ';' before 'bar'  ← Might be caused by first error
error: 'bar' not declared         ← Probably caused by above
```

### Find the actual line

Error messages tell you where:
```
src/main.cpp:42:15: error: ...
     ^
     │
     └── Line 42, column 15
```

### Verbose output

```bash
pros make VERBOSE=1
```

Shows full compilation commands.

---

## Quick Reference

| Error | Likely Cause | Fix |
|-------|--------------|-----|
| `file not found` | Wrong path | Check file location and spelling |
| `undefined reference` | Missing implementation | Add .cpp implementation |
| `not a member of` | Wrong class/spelling | Check spelling and class |
| `does not name a type` | Missing include | Add #include |
| `redefinition` | Defined twice | Use #pragma once, move to .cpp |
| `expected ';'` | Missing semicolon | Check previous line |
| `cannot convert` | Type mismatch | Check parameter/return types |
| `undeclared identifier` | Variable not declared | Declare or check scope |

---

*For other issues, see [COMMON_ISSUES.md](./COMMON_ISSUES.md)*