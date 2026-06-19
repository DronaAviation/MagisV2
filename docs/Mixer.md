# Mixer Configuration

The MagisV2 PRIMUS architecture has drastically simplified the mixer engine to save computational overhead and flash memory space. 

As a result, **only Quadcopter mixing geometries are supported natively**.

## Supported Mixer Types

| Name             | Description               | Motors         | Servos           |
| ---------------- | ------------------------- | -------------- | ---------------- |
| QUADP            | Quadcopter-Plus           | M1-M4          | None             |
| QUADX            | Quadcopter-X              | M1-M4          | None             |

## Deprecation Notice

> [!WARNING]
> **DEPRECATED**: The legacy `CUSTOM` mixer table engine (`mmix` and `smix` CLI commands) and all non-quadcopter geometries (e.g., `TRI`, `HEX6`, `OCTOX8`, `AIRPLANE`, `BI`) have been permanently disabled and purged from the MagisV2 core.
> 
> Attempting to load these configurations will result in a failure mode or fallback to `QUADX`.

## Standard Quadcopter Layouts

### QUADX
This is the default and standard layout for modern FPV racing drones.
```
    4CW      2CCW
       \    /
        FC
       /    \
    3CCW     1CW
```

### QUADP
```
         2CCW
          |
 4CW --- FC --- 1CW
          |
         3CCW
```

## Motor Filtering and Tuning
While complex geometric mixing has been removed, individual motor outputs are still subjected to the core PID control loops and dynamic filtering mechanisms. For more details on how mathematical corrections are applied to these geometries, refer to the [Mixer Pipeline Architecture](fw-architecture-pipeline/subsystems/Mixer_Pipeline.md) documentation.
