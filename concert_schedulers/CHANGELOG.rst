=========
Changelog
=========

0.6.9 (2015-07-09)
------------------
* ros friendly name matching for releasing client closes `#307 <https://github.com/robotics-in-concert/rocon_concert/issues/307>`_
* Contributors: Jihoon Lee

0.6.8 (2015-04-27)
------------------

0.6.7 (2015-04-06)
------------------

0.6.6 (2015-03-23)
------------------

0.6.5 (2015-02-27)
------------------

0.6.4 (2015-02-09)
------------------

0.6.3 (2015-01-05)
------------------
* update install rule fixes `#253 <https://github.com/robotics-in-concert/rocon_concert/issues/253>`_
* Contributors: Jihoon Lee

0.6.2 (2014-12-02)
------------------

0.6.1 (2014-11-21)
------------------
* [rocon_concert] permit esoteric names once more.
* Contributors: Daniel Stonier

0.6.0 (2014-08-25)
------------------
* parameter enabled link graph
* concert_schedulers sphinxed.
* comment explaining the big lock effect.
* better feedback on cancelled requests.
* Merge branch 'hydro-devel' into indigo
* logging to trace cancelling requests, `#231 <https://github.com/robotics-in-concert/rocon_concert/issues/231>`_.
* enable preemption as an option in the compatibility tree scheduler.
* update publisher queue_size to avoid warning in indigo.
* complicated lock handling, fixes `#227 <https://github.com/robotics-in-concert/rocon_concert/issues/227>`_
* compatibility tree scheduler now tolerates missing clients.
* bugfixes to reenable chatter concert.
* client status/gateway-info update handling and alot of minor fixes.
* compatibility tree now favours unallocated clients, closes `#218 <https://github.com/robotics-in-concert/rocon_concert/issues/218>`_
* reworked to enable preemptions
* usable configuration for service priorities.
* rocon_scheduler_requests -> concert_scheduling/concert_scheduler_requests
* update the readme link.
* reworked scheduler for the simple scheduler and a repeat allocation bugfix for the compatibility tree scheduler.
* automagically finding the scheduler requests topic.
* integrating jack's scheduler.
* bugfix logging in rare exception.
* rocon_scheduler_requests -> concert_scheduler_requests
* scheduler launchers.
* reloading of solution configuration and service profiles as needed.
* remove debug logging.
* known_resources -> resource_pool to fit with jacks naming.
* publishing by convention a known_resources topic now.
* typo fix.
* interactions moved to rocon_tools, also updates for the rocon_utilities breakup.
* remove last vestiges of the demo scheduler.
* rapp_name -> rapp in line with Resource.msg style
* bugfix for updated request flag, RELEASED->CLOSED
* bugfix the concert scheduler for parallel requesters.
* updates for the new rocon uri
* platform tuple overhaul.
* update for recently moved modules to rocon_tools, also platform_tuples refactoring.
* fix compatibility tree unit test.
* update for new scheduler msg api.
* using jack's latest bugfix, external cancel/new request ok.
* able to add/remove optional resources now.
* cancelling the pool requests if dropping below the minimal status.
* Scan dynamically and continuously for scheduler request topics.
* use jack's new free() scheduler reply api for releasing requests.
* attempt to handle lost clients, but not able to feedback from scheduler to requester yet.
* doc comments.
* states for the requester.
* detect client changes in the scheduler.
* enable/disable services with new scheduler working, also debugging scheduler requests script.
* catch and handle exception when all apps do not start.
* catch and handle exception when all apps do not start.
* bugfix incorrect accumulation of requests.
* demo scheduler/requester enable/disable services back in.
* infrastructure for releasing resources on the requester side.
* wait status update for new requests.
* bugfix service connection errors and delays from rogue clients in the conductor.
* remove comment
* bugfix error recursion process in demo scheduler.
* remove debugging information.
* bugfix double nested break, also repeat check allocation attempts for when they fail via periodic concert clients listener.
* bugfix double nested break, also repeat checks via periodic concert clients listener in case an allocation failed.
* trivial comment change.
* trim and clean up logging information.
* accept remote gateways with ip 'localhost' as local clients.
* compatibility tree scheduler and resource pool requester (very rough).
* starting work on the minmax request/requester.
* upgrade for ConcertClient platform info format change.
* very skeletal, but working scheduler for adding chatter clients.
* minor infringements...
* starting apps in the compatibility tree scheduler.
* working towards the compatibility tree scheduler.
* Contributors: Daniel Stonier, Jihoon Lee

* launcher for Jack's simple scheduler.
* compatibility tree scheduler with preemption variant.
* compatibility tree scheduler implementation.
* resource pool requester implementation.
