# HCI-FACE Front End

The design of the face is based on the CoRDial framework. The app recieves server sent events from the backend API in order to know what expressions to make. The faceControl api can send expressions or behaviors, while visemes are process seperately to allow talking and expression to take place at the same time.

There are currently three faces available: default, cordial, and qt. You can choose the face by changing the face option in the Head component in the App.js:

``` <Head face="cordial" ... \> ```

There are three main components of the face. The rendering, the animation, and the control.

## Rendering

Rendering is where the head is drawn in SVG. The eyes, brows, and mouth are rendered seperately based on the facial action units. The face adjusts the location of the drawing to new action unit positions by interpolating with the react-spring library.

## Control

Control provides a form input with sliders for all available action units. Not all faces take advantage of all action units.

## Animation

Animation is where visemes, expressions, and behaviors are encoded into the action units that will be consumed by the face. The visemes and expressions are static, but behaviors is based on repeating loops of expressions or eye movements.
