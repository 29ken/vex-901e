#pragma once

// Call both once from initialize().
void startBrainScreen();
void startControllerScreen();

// True while the controller menu is open. A is the menu's + button then, so
// the driver loop shouldn't use it for the claw.
bool controllerMenuOpen();

// Switches the brain screen to its Distance page with this tape distance
// (inches), as the middle button and left/right do. For the simulator's check.
void showDistancePage(int tapeInches);
