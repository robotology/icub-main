# embObjMotionControl Parser Maintenance Guide

## Purpose

This guide explains how to safely extend and maintain the embObjMotionControl parser after the YARP4 modernization work.

The goals of the refactor are:
- improve maintainability and readability,
- keep behavior explicit and predictable,
- reduce noisy startup logs while preserving full configuration visibility.

## Architecture Intent

The parser architecture follows a clear separation of concerns:
- Types and configuration payloads are defined in embObjMotionControlConfigTypes.h.
- Default values are centralized in embObjMotionControlDefaults.h.
- Parsing, validation, and assignment logic are implemented in eomcParser.cpp and declared in eomcParser.h.

A key design point is optional-parameter tracking through registerOptionalParameter().

Instead of flooding yarprobotinterface output with repeated messages such as "parameter X missing, using default Y" for each missing key, the parser records optional/defaulted parameters and prints one consolidated summary table through printOptionalParametersTable().

This keeps logs readable and makes users more aware of the effective configuration at a glance.

## Files To Update When Adding Or Changing Parameters

When adding a new parser parameter, or changing an existing one, update all applicable layers:

1. Type/schema layer
- File: src/libraries/icubmod/embObjMotionControl/embObjMotionControlConfigTypes.h
- Action: add or update the typed field in the relevant struct.

2. Default-value layer
- File: src/libraries/icubmod/embObjMotionControl/embObjMotionControlDefaults.h
- Action: define or update the default value in the appropriate defaults block.

3. Parser layer
- File: src/libraries/icubmod/embObjMotionControl/eomcParser.cpp
- Action: parse the key, validate it, assign it, and track optional/default usage with registerOptionalParameter() when relevant.

4. Parser interface layer
- File: src/libraries/icubmod/embObjMotionControl/eomcParser.h
- Action: update declarations if parser data flow or signatures change.

5. Documentation layer
- File: src/libraries/icubmod/embObjMotionControl/embObjMotionControlConfigurationDefaults.md
- Action: update documentation for new or changed defaults and expected keys.

## Standard Parser Pattern To Reuse

Use this order consistently:

1. Read the group
- Use GetGroupBottle() for mandatory/optional group retrieval.

2. Read parameter values
- Use extractGroup() for joint-wise vectors/lists.
- For scalar values, use Bottle find checks and type checks.

3. Validate early
- Verify type, cardinality, enum compatibility, and range.
- Fail fast with board-aware and joint-aware error messages.

4. Handle optional parameters
- If missing and optional, assign default from _defaultSettings.
- Call registerOptionalParameter(parameterName, defaultValue, wasUsed).

5. Assign final values
- Write into typed per-joint/per-group destination structures.

6. Report once
- Avoid repeated warning spam for every missing key.
- Use the consolidated optional-parameter summary table.

## Optional Parameter Logging Policy

Preferred behavior:
- Mandatory missing or invalid parameter: fail and report clear error.
- Optional missing parameter: apply default and register it.
- End of parsing/open: print a single summary table of optional/defaulted parameters.

Benefits:
- cleaner logs,
- easier troubleshooting,
- better user awareness of effective runtime configuration.

## Backward Compatibility Policy

When renaming or changing semantics of a parameter:
- keep old key support for one transition cycle when possible,
- parse new key first and old key as fallback,
- emit a single compatibility warning (not repeated per joint),
- document deprecation and removal timeline.

## Validation Checklist For Pull Requests

Use this checklist before merging parser changes:

1. Type exists and is correctly placed in embObjMotionControlConfigTypes.h.
2. Default exists and is correctly placed in embObjMotionControlDefaults.h.
3. Parser reads key and validates value in eomcParser.cpp.
4. Optional parameter handling uses registerOptionalParameter() where applicable.
5. Error messages include board context and joint context when relevant.
6. No noisy per-parameter default warnings were added.
7. Summary-table behavior remains intact.
8. Related docs were updated.
9. Existing robot configuration compatibility was considered.
10. The corresponding template for the target version was updated in robots-configuration iCub templates: https://github.com/robotology/robots-configuration/tree/master/iCubTemplates.

## Common Pitfalls

- Adding parsing logic without adding typed storage.
- Adding typed storage without adding defaults.
- Using hardcoded magic defaults in parser code.
- Logging one warning per joint for the same optional key.
- Forgetting consistency checks across JOINTSET_CFG constraints.
- Changing key names without compatibility fallback.
