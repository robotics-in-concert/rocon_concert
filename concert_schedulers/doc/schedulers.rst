==========
Schedulers
==========

Overview
--------

For each scheduler implementation, this package has a launcher that can be triggered
by arg in the concert master launcher. e.g.

.. code-block:: bash

   > roslaunch concert_master concert_master.launch scheduler_type:=preemptive_compatibility_tree

Each launcher has the form ``<scheduler_name>_scheduler.launch`` and the
current default is ``compatibility_tree``.

Scheduling is not trivial. A simple example to illustrate - suppose we require three
separate resources for a single request and we have clients with names 'a'-'g' available.

* ``rocon_apps/patrol``    : a b c d e f
* ``rocon_apps/deliver``   : a b c d e g
* ``rocon_apps/clean``     : f g

Here, every concert client can satisfy two different resources. Note that if we take
randomly select f and g for the first two resource allocations, then the third is unfilled
and the request unresolved. Here the obvious approach is to appropriately allocate f or g to
the third resource due to the solutions sensitivity to this resource (fewest number of allocatable
clients). This is exactly the approach the compatibility tree scheduler adopts.

We could also exhaustively search all permutations, but even then
we'd need a good metric to ascertain what is the *best* allocation.

Simple Scheduler
----------------

This scheduler is provided by the `concert_simple_scheduler`_ package and will be the default once
fully featured. It uses a set of simple heuristics to choose an appropriate action from the
many permutations that can arise when satisfying multiple requests of multiple resources.

.. _`concert_simple_scheduler`: http://wiki.ros.org/concert_simple_scheduler

Compatibility Tree
------------------

This is the experimental default and does a similar job to the simple scheduler using a
``compatibility tree`` algorithm. This is implemented in the
:class:`.concert_schedulers.compatibility_tree_scheduler.scheduler.CompatibilityTreeScheduler` class.

Preemptive Compatibility Tree
-----------------------------

This adds an experimental hook to allocate from already allocated clients that are currently
working at a lower priority. It immediately pulls them, stops their rapp and starts their new
rapp (task). This is as *ungraceful* as a hippo in a ballet class [#f1]_, but it's a first step to
experimenting with preemptions.

This is also implemented in the
:class:`.concert_schedulers.compatibility_tree_scheduler.scheduler.CompatibilityTreeScheduler` class
but triggered by the ``enable_preemptions`` parameter.

.. [#f1] It should set a timeout, notify the service that is requesting and let the service cancel the request gracefully. If the service fails to do so, only then should it curtly stop the rapp and proceed.
