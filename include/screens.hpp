#pragma once

// Call both once from initialize().
void startBrainScreen();
void startControllerScreen();

// True while the controller menu is open. The driver loop does not use A (the
// menu's + button), so nothing needs this now; kept for a button that might share one.
bool controllerMenuOpen();
