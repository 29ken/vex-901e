#pragma once

// Call both once from initialize().
void startBrainScreen();
void startControllerScreen();

// True while the controller menu is open. A is the menu's + button then, so
// the driver loop shouldn't use it for the claw.
bool controllerMenuOpen();
