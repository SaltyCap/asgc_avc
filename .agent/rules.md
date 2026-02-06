# Agent Coding Rules

## Core Principles

1. **Safety First**: Always verify changes won't break existing functionality
2. **Clarity Over Cleverness**: Write code that is easy to understand and maintain
3. **Incremental Changes**: Make small, testable changes rather than large rewrites
4. **Verify Everything**: Test and validate all modifications before considering them complete

---

## Code Formatting

### General Formatting
- Use consistent indentation (match the existing codebase style)
- Keep line length reasonable (typically 80-120 characters)
- Use whitespace to separate logical blocks of code
- Place opening braces on the same line for functions (K&R style) or next line (Allman style) - **be consistent with existing code**
- Add blank lines between function definitions

### Naming Conventions
- **Functions**: Use descriptive verb phrases (e.g., `calculate_distance`, `init_sensor`, `process_command`)
- **Variables**: Use descriptive nouns that explain purpose (e.g., `sensor_reading`, `max_speed`, `current_position`)
- **Constants**: Use UPPER_CASE_WITH_UNDERSCORES (e.g., `MAX_BUFFER_SIZE`, `DEFAULT_TIMEOUT`)
- **Avoid**: Single-letter variables except for loop counters (`i`, `j`, `k`) in short loops
- **Avoid**: Abbreviations unless they are well-known in the domain (e.g., `pwm`, `i2c`, `uart`)

### File Organization
- Group related functions together
- Place includes at the top of files
- Order sections logically: constants → types → globals → function declarations → implementations
- Keep related code in the same file when possible

---

## Comments and Documentation

### When to Comment
- **Always**: Explain WHY, not WHAT the code does
- **Always**: Document non-obvious behavior, edge cases, or workarounds
- **Always**: Add function headers for public APIs
- **Never**: State the obvious (e.g., `i++; // increment i`)

### Comment Style
```c
/**
 * Brief one-line description of function purpose
 * 
 * Detailed explanation if needed, including:
 * - Important parameters and their valid ranges
 * - Return value meaning
 * - Side effects or state changes
 * - Thread safety considerations
 * 
 * @param param_name Description of parameter
 * @return Description of return value
 */
int example_function(int param_name) {
    // Inline comments explain WHY decisions were made
    // Example: "Using busy-wait here because sleep() has 10ms granularity"
    
    return 0;
}
```

### Inline Comments
- Place above the code block they describe, not at the end of lines (unless very brief)
- Explain complex algorithms or non-obvious logic
- Document magic numbers with named constants and comments
- Mark TODOs clearly: `// TODO: Add error handling for timeout case`

---

## Variable Management

### Before Changing Variables

1. **Search for all references** using grep/search tools across the entire codebase
2. **Check header files** for declarations and external references
3. **Verify scope**: Understand if the variable is local, global, or shared across threads
4. **Check for macros** that might reference the variable name

### Renaming Variables

1. **Use find-and-replace carefully**: Ensure you're not accidentally changing unrelated code
2. **Update all references**: Code, comments, documentation, and log messages
3. **Verify compilation**: Ensure code compiles without warnings after changes
4. **Test functionality**: Run tests or manual verification to confirm behavior is unchanged

### Variable Scope Best Practices
- **Minimize scope**: Use the smallest scope possible (prefer local over global)
- **Initialize variables**: Always initialize variables at declaration when possible
- **Const correctness**: Use `const` for variables that shouldn't change
- **Avoid globals**: Use globals only when necessary; prefer passing parameters

---

## Code Modification Workflow

### Before Making Changes

1. **Understand the current behavior**: Read and comprehend existing code
2. **Identify dependencies**: Find all code that depends on what you're changing
3. **Plan the change**: Think through implications before editing
4. **Check for tests**: Look for existing tests that validate current behavior

### During Changes

1. **Make minimal edits**: Change only what's necessary to achieve the goal
2. **Preserve existing behavior**: Unless explicitly changing functionality, maintain current behavior
3. **Update related code**: If changing a function signature, update all call sites
4. **Update comments**: Keep comments in sync with code changes

### After Changes

1. **Verify compilation**: Ensure code compiles without errors or warnings
2. **Check for regressions**: Test that existing functionality still works
3. **Run tests**: Execute any automated tests or verification scripts
4. **Manual testing**: Perform manual verification of changed functionality
5. **Review logs**: Check for unexpected errors or warnings in runtime logs

---

## Verification Requirements

### Compilation Verification
- **Always compile** after making changes
- **Fix all warnings**: Treat warnings as errors
- **Check build output**: Look for deprecation notices or other issues

### Runtime Verification
- **Test the specific change**: Verify the modified functionality works as intended
- **Test related features**: Ensure dependent features still work correctly
- **Monitor for errors**: Check logs, console output, and error messages
- **Performance check**: Ensure changes don't introduce performance regressions

### Cross-Reference Verification
Before modifying any symbol (function, variable, constant):
1. Use `grep` or search tools to find all occurrences
2. Check header files for declarations
3. Verify no external modules depend on it
4. Confirm the scope and visibility (static, extern, etc.)

---

## Error Handling

### General Principles
- **Check return values**: Always check return values from functions that can fail
- **Handle errors gracefully**: Don't ignore errors; log them or propagate them
- **Fail safely**: Ensure system reaches a safe state on error
- **Provide context**: Error messages should include enough context to debug

### Error Handling Patterns
```c
// Good: Check return value and handle error
int result = risky_operation();
if (result != SUCCESS) {
    log_error("Operation failed: %d", result);
    cleanup_resources();
    return ERROR_CODE;
}

// Bad: Ignoring return value
risky_operation();  // What if this fails?
```

---

## Multi-threaded Code

### Thread Safety Considerations
- **Document thread safety**: Clearly indicate if functions are thread-safe
- **Protect shared data**: Use mutexes, locks, or atomic operations for shared variables
- **Avoid race conditions**: Be careful with read-modify-write operations
- **Minimize lock scope**: Hold locks for the shortest time possible

### Synchronization
- **Use appropriate primitives**: Mutexes for mutual exclusion, condition variables for signaling
- **Avoid deadlocks**: Always acquire locks in the same order
- **Document locking order**: Comment on lock acquisition order if multiple locks are used

---

## Performance Considerations

### Optimization Guidelines
- **Profile first**: Don't optimize without measuring
- **Readability over micro-optimization**: Clear code is better than slightly faster obscure code
- **Optimize algorithms**: Focus on algorithmic improvements over micro-optimizations
- **Document performance-critical sections**: Mark hot paths with comments

### Resource Management
- **Free allocated memory**: Always free what you allocate
- **Close file handles**: Ensure resources are released properly
- **Avoid leaks**: Use tools like valgrind to detect memory leaks
- **Limit resource usage**: Be mindful of memory, CPU, and I/O usage

---

## Testing and Validation

### Before Committing Changes
- [ ] Code compiles without errors or warnings
- [ ] All existing tests pass
- [ ] New functionality is tested (automated or manual)
- [ ] No regressions in related features
- [ ] Code follows project style guidelines
- [ ] Comments and documentation are updated
- [ ] All TODOs are addressed or documented

### Testing Checklist
1. **Unit level**: Does the modified function work correctly?
2. **Integration level**: Do components work together correctly?
3. **System level**: Does the overall system behave as expected?
4. **Edge cases**: Are boundary conditions and error cases handled?

---

## Common Pitfalls to Avoid

### Code Quality
- ❌ Don't use magic numbers - use named constants
- ❌ Don't write overly complex functions - break them down
- ❌ Don't duplicate code - extract common functionality
- ❌ Don't leave dead code - remove unused functions and variables
- ❌ Don't ignore compiler warnings - fix them

### Safety
- ❌ Don't assume success - always check return values
- ❌ Don't use uninitialized variables
- ❌ Don't access arrays out of bounds
- ❌ Don't dereference NULL pointers
- ❌ Don't create race conditions in multi-threaded code

### Maintainability
- ❌ Don't write cryptic code - clarity is key
- ❌ Don't skip documentation for complex logic
- ❌ Don't make global changes without verification
- ❌ Don't change APIs without updating all callers
- ❌ Don't commit untested code

---

## Agent-Specific Instructions

### When Making Code Changes
1. **Always search for references** before renaming or removing variables/functions
2. **Verify compilation** after every significant change
3. **Propose testing steps** to validate changes
4. **Ask for clarification** if requirements are ambiguous
5. **Explain trade-offs** when multiple approaches are possible

### Communication
- **Explain what you're doing**: Describe changes before making them
- **Highlight risks**: Point out potential issues or breaking changes
- **Suggest verification**: Recommend how to test and validate changes
- **Be transparent**: Acknowledge when you're unsure or need more information

### Workflow
1. Understand the request fully
2. Analyze existing code and dependencies
3. Plan the changes and identify risks
4. Make minimal, focused edits
5. Verify compilation and functionality
6. Suggest testing and validation steps
