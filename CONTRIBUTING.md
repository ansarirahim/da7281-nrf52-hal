# Contributing to DA7281 HAL

Thank you for your interest in contributing to this project!

## Code Style

### C Coding Standards

This project follows professional embedded C coding standards:

#### Naming Conventions

- **Functions**: `module_action_object()` format
  ```c
  da7281_set_operation_mode()
  da7281_read_register()
  ```

- **Variables**: `snake_case`
  ```c
  uint8_t reg_value;
  da7281_device_t haptic_dev;
  ```

- **Constants/Macros**: `UPPER_SNAKE_CASE`
  ```c
  #define DA7281_REG_TOP_CFG1  (0x22U)
  #define DA7281_POWER_UP_DELAY_MS  (2U)
  ```

- **Types**: `module_name_t` suffix
  ```c
  typedef struct { ... } da7281_device_t;
  typedef enum { ... } da7281_error_t;
  ```

#### File Organization

Each source file should have:
```c
/**
 * @file filename.c
 * @brief Brief description
 * @author Author Name
 * @date YYYY-MM-DD
 * @version X.Y.Z
 * 
 * @copyright Copyright notice
 * 
 * Detailed description...
 */

/*===========================================================================*/
/* INCLUDES                                                                  */
/*===========================================================================*/

/*===========================================================================*/
/* PRIVATE MACROS AND DEFINES                                                */
/*===========================================================================*/

/*===========================================================================*/
/* PRIVATE VARIABLES                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                               */
/*===========================================================================*/

/*===========================================================================*/
/* PUBLIC FUNCTION IMPLEMENTATIONS                                           */
/*===========================================================================*/

/*===========================================================================*/
/* PRIVATE FUNCTION IMPLEMENTATIONS                                          */
/*===========================================================================*/

/*===========================================================================*/
/* END OF FILE                                                               */
/*===========================================================================*/
```

#### Documentation

All public functions must have Doxygen comments:
```c
/**
 * @brief Brief description of function
 * 
 * Detailed description if needed.
 * 
 * @param[in] param1 Description of input parameter
 * @param[out] param2 Description of output parameter
 * @param[in,out] param3 Description of in/out parameter
 * @return Return value description
 * @retval DA7281_OK Success
 * @retval DA7281_ERR_INVALID_PARAM Invalid parameter
 */
da7281_error_t da7281_function_name(const type *param1, type *param2);
```

#### Error Handling

- All functions return error codes
- Use `const` for read-only parameters
- Validate all parameters at function entry
- Clean up resources on error

```c
da7281_error_t da7281_example(const da7281_device_t *dev, uint8_t value)
{
    // Validate parameters
    if (dev == NULL) {
        return DA7281_ERR_INVALID_PARAM;
    }
    
    if (!dev->is_powered) {
        return DA7281_ERR_NOT_INITIALIZED;
    }
    
    // Implementation...
    
    return DA7281_OK;
}
```

## Commit Message Format

Follow this format for commit messages:

```
<type>: <subject>

<body>

<footer>
```

### Types
- `HAL`: Core HAL implementation changes
- `Examples`: Example code changes
- `Docs`: Documentation updates
- `Build`: Build system changes
- `CI`: CI/CD pipeline changes
- `Fix`: Bug fixes
- `Feature`: New features
- `Refactor`: Code refactoring

### Examples

```
HAL: Add support for PWM mode operation

- Implement PWM frequency configuration
- Add duty cycle control functions
- Update register definitions for PWM mode
- Include timing calculations per datasheet

Tested on DWM3001CDK with 200Hz PWM
```

```
Fix: Correct I2C mutex timeout handling

The I2C mutex was not being released on timeout,
causing subsequent operations to hang.

- Release mutex in all error paths
- Add timeout logging
- Update error code documentation

Fixes #42
```

## Pull Request Process

1. **Fork the repository**
2. **Create a feature branch**
   ```bash
   git checkout -b feature/your-feature-name
   ```

3. **Make your changes**
   - Follow code style guidelines
   - Add/update documentation
   - Add/update tests if applicable

4. **Test your changes**
   - Build successfully
   - Run static analysis
   - Test on hardware if possible

5. **Commit with clear messages**
   ```bash
   git commit -m "HAL: Add your feature description"
   ```

6. **Push to your fork**
   ```bash
   git push origin feature/your-feature-name
   ```

7. **Create Pull Request**
   - Describe what changed and why
   - Reference any related issues
   - Include test results

## Code Review Checklist

Before submitting, ensure:

- [ ] Code follows naming conventions
- [ ] All functions have Doxygen comments
- [ ] No magic numbers (use named constants)
- [ ] Error handling is complete
- [ ] Memory is properly managed
- [ ] Thread safety is maintained
- [ ] Datasheet timing requirements are met
- [ ] Code compiles without warnings
- [ ] Documentation is updated

## Questions?

Feel free to open an issue for:
- Bug reports
- Feature requests
- Documentation improvements
- General questions

Thank you for contributing!

