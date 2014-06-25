.. concert_schedulers documentation master file, created by
   sphinx-quickstart on Mon Apr 21 16:50:17 2014.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Concert Schedulers
==================

The `concert_schedulers` package maintains a couple of simple implementations of
requesters and schedulers built on top of the `concert_scheduler_requests`_ infrastructure.
It also exposes these, and others that can be found in external packages to the concert
via launchers that can be triggered by args passed to the `concert_master`_.

.. _`concert_master`: http://wiki.ros.org/concert_master
.. _`concert_scheduler_requests`: http://wiki.ros.org/concert_scheduler_requests
.. _`concert_schedulers`: http://wiki.ros.org/concert_schedulers
.. _`Robotics in Concert`: http://www.robotconcert.org/wiki/Main_Page

.. toctree::
   :maxdepth: 2

   requesters
   schedulers
   modules
   changelog

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

