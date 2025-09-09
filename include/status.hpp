#pragma once

// Inicializa a task de status do LED.
// Mantém LED aceso quando transmitindo; pisca quando o link está ocioso
// (rápido se conectado, lento se desconectado).
void statusInit();
