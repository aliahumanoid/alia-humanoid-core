#ifndef MCP_CAN_GUARD_H
#define MCP_CAN_GUARD_H

#ifndef MCP_CAN_GUARD_APPLIED
#define MCP_CAN_GUARD_APPLIED

#ifdef MCP2515_SELECT
#undef MCP2515_SELECT
#endif
#ifdef MCP2515_UNSELECT
#undef MCP2515_UNSELECT
#endif

// Simple SPI1 access - NO LOCK NEEDED
// Core1 has exclusive access to CAN bus (both Host and Motor)
// No conflicts possible with this architecture
#define MCP2515_SELECT()  do { digitalWrite(MCPCS, LOW); } while (0)
#define MCP2515_UNSELECT() do { digitalWrite(MCPCS, HIGH); } while (0)

#endif // MCP_CAN_GUARD_APPLIED

#endif // MCP_CAN_GUARD_H

