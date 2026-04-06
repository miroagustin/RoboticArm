# Extending To Another Arm

Esta base esta pensada para que el brazo de referencia sea intercambiable sin reescribir el agente.

## Regla principal

El contrato MCP es estable. Si cambia el hardware, el agente deberia seguir funcionando con cambios minimos o nulos.

## Puntos de extension

### Backend del brazo

Crear un backend nuevo dentro de `robotic-arm-mcp/backends/` que implemente la interfaz comun:

- `get_capabilities()`
- `get_position()`
- `move_joints()`
- `move_cartesian()`
- `go_home()`
- `emergency_stop()`

Si el brazo soporta features extra, pueden exponerse como tools opcionales siempre que `get_capabilities()` lo anuncie.

### Safety

Actualizar `robotic-arm-mcp/safety.py` para reflejar:

- limites articulares reales
- workspace seguro
- velocidad maxima permitida
- reglas de parada y rate limiting

### Perception y planning

`robotic-arm-mcp/perception/` y `robotic-arm-mcp/backends/planning.py` se pueden reemplazar o extender segun la nueva celda robotica.

### Deploy

Si el nuevo brazo necesita drivers, simuladores o assets propios:

- agrega una variante en `deploy/`
- documenta las variables nuevas en `.env.example`
- actualiza `deploy/README.md`

## Estrategia de fork

Si el nuevo brazo implica cambios fuertes de ROS, firmware o vendor SDK, la opcion recomendada es forkar este repo y mantener:

- `robotic-arm-agent/` casi sin cambios
- el contrato MCP compatible
- un `robotic-arm-mcp/` adaptado al hardware objetivo

Eso permite conservar el mismo flujo de agente y despliegue mientras cambia solo la implementacion robotica.
