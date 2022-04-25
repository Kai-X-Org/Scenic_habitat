..  _specifiers:

********************
Specifiers Reference
********************

Position Specifiers
===================

.. _at *vector*:

at *vector*
-----------
Positions the object at the given global coordinates.

.. _offset by *vector*:

offset by *vector*
------------------
Positions the object at the given coordinates in the local coordinate system of ego (which must already be defined).

.. _offset along *direction* by *vector*:

offset along *direction* by *vector*
------------------------------------
Positions the object at the given coordinates, in a local coordinate system centered at ego and oriented along the given direction (which, if a vector field, is evaluated at ego to obtain a heading).

.. _(left | right) of *vector* [by *scalar*]:

(left | right) of *vector* [by *scalar*]
----------------------------------------
Depends on heading and width. Without the optional by scalar, positions the object immediately to the left/right of the given position; i.e., so that the midpoint of the object’s right/left edge is at that position. If by scalar is used, the object is placed further to the left/right by the given distance.

.. _(ahead of | behind) *vector* [by *scalar*]:

(ahead of | behind) *vector* [by *scalar*]
--------------------------------------------
As above, except placing the object ahead of or behind the given position (so that the midpoint of the object’s back/front edge is at that position); thereby depending on heading and length.

.. _beyond *vector* by *vector* [from *vector*]:

beyond *vector* by *vector* [from *vector*]
--------------------------------------------
Positions the object at coordinates given by the second vector, in a local coordinate system centered at the first vector and oriented along the line of sight from the third vector (i.e. a heading of 0 in the local coordinate system faces directly away from the first vector). If no third vector is provided, it is assumed to be the ego. For example, beyond taxi by 0 @ 3 means 3 meters directly behind the taxi as viewed by the camera.

.. _visible [from (*Point* | *OrientedPoint*)]:

visible [from (*Point* | *OrientedPoint*)]
------------------------------------------
Positions the object uniformly at random in the visible region of the ego, or of the given Point/OrientedPoint if given. Visible regions are defined as follows: a Point can see out to a certain distance, and an OrientedPoint restricts this to the circular sector along its heading with a certain angle. A position is then visible if it lies in the visible region and the position of the object being specified is set such that the center of the object is in this visible region.

.. _(in | on) *region*:

(in | on) *region*
------------------
Positions the object uniformly at random in the given Region. If the Region has a preferred orientation (a vector field), also optionally specifies heading to be equal to that orientation at the object’s position.

.. _(left | right) of (*OrientedPoint* | *Object*) [by *scalar*]:

(left | right) of (*OrientedPoint* | *Object*) [by *scalar*]
------------------------------------------------------------
Positions the object to the left/right of the given OrientedPoint, depending on the object’s width. Also optionally specifies heading to be the same as that of the OrientedPoint. If the OrientedPoint is in fact an Object, the object being constructed is positioned to the left/right of its left/right edge.

.. _(ahead of | behind) (*OrientedPoint* | *Object*) [by *scalar* ]:

(ahead of | behind) (*OrientedPoint* | *Object*) [by *scalar* ]
---------------------------------------------------------------
As above, except positioning the object ahead of or behind the given OrientedPoint, thereby depending on length

.. _following *vectorField* [from *vector* ] for *scalar*:

following *vectorField* [from *vector* ] for *scalar*
-----------------------------------------------------
Positions the object at a point obtained by following the given vector field for the given distance starting from ego (or the position optionally provided with from vector ). Optionally specifies heading to be the heading of the vector field at the resulting point. Uses a forward Euler approximation of the continuous vector field


Heading Specifiers
==================

.. _facing *heading*:

facing *heading*
----------------
Orients the object along the given heading in global coordinates

.. _facing *vectorField*:

facing *vectorField*
--------------------
Orients the object along the given vector field at the object’s position

.. _facing (toward | away from) *vector*:

facing (toward | away from) *vector*
------------------------------------
Orients the object toward/away from the given position (thereby depending on the object’s position)

.. _apparently facing *heading* [from *vector*]:

apparently facing *heading* [from *vector*]
--------------------------------------------
Orients the object so that it has the given heading with respect to the line of sight from ego (or from the position given by the optional from vector). For example, apparently facing 90 deg orients the object so that the camera views its left side head-on