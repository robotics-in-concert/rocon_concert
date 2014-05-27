.. concert_conductor documentation master file, created by
   sphinx-quickstart on Mon Apr 21 16:50:17 2014.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Concert Conductor
=================

The `concert_conductor` is responsible for inviting and managing connections between
a `Robotics in Concert`_ concert and it's concert clients. Part of this responsibility
involves introspecting and exposing potential client information to the concert decision
makers, and another is handling the invitation and status of connected clients.

Note that it is not responsible for running anything in particular, nor making any
decisions about who should/should not join or execute code.

.. _`concert_conductor`: http://wiki.ros.org/concert_conductor
.. _`Robotics in Concert`: http://www.robotconcert.org/wiki/Main_Page

.. toctree::
   :maxdepth: 2

   introspection
   specifications
   modules
   changelog

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

