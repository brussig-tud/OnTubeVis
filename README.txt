__________________________________
>>> Disclaimer <<<
The application was tested on systems equipped with relatively modern discrete graphics hardware (Nvidia GTX 680 and RTX 2080/Ti).
We could not test the application on AMD graphic cards. Running the application on notebooks that only have an integrated graphics
processor may not work as intended.
____________________________________________________

Load data sets and configurations by drag & drop or through the GUI.
Configurations depend on attributes and therefore only work for their respective data set.

View interaction:
  - use the mouse pointer to rotate the view
  - hold right mouse button to pan
  - hold shift to tilt
  - click on any geometry to set the view focus
  - scroll to zoom in, when scrolling while the pointer is over some geometry, the view will zoom in to that position

Keyboard shortcuts:
  , - reset view
  . - reset view closer (not actually the far and near configurations used during benchmarks)
  A - toggle ambient occlusion
  G - cycle grid modes
  M - toggle color maps widget
  O - toggle boundign box
  R - double the radius; hold shift to half the radius
  W - toggle wireframe box
  F12 - toggle background colors

Color scales:
  To add a color map, open the "Color Scales" GUI group. Enter a name that is not already taken and press "Add Color Map".
  The color scale will appear at the bottom of the list. Press the pencil icon rigt next tot he name to open the editor.
  Preset color scales cannot be edited. To add control points, hold CTRL and click with the mouse. Hold ALT and click on
  a control grip to delete it. Change colors in the GUI.

Additional considerations:
You might encounter pixel artifacts at segment boundaries, because:
The intersection routine is based on an iterative root solver. The amount of iterations directly determines the precision of the
intersection. This is noticeable at the overlaps of two consecutive segments. Since the depth value can be inprecise, z-fighting
might occur. Further this also depends on the near and far clipping plane settings. To mitigate this somewhat, the spherical
segment caps are hidden at a certain view distance. This explains the popping of segment caps you will probably encounter.

Solution:
  For the data sets with a large scene extent (sepia, nox), increase the "Cap Clip Distance" under the "Tube Style" GUI group.
  You can enter values greater than 100 directly in the value input field. You might need to enter 10000.
  Click on "Stereo Interactor" tab and at the bottom increase the near clipping distance "z Near".