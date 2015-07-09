=========
Changelog
=========

0.6.9 (2015-07-09)
------------------
* add transition available to uninvited `#302 <https://github.com/robotics-in-concert/rocon_concert/issues/302>`_
* callable error fix
* update transition table `#302 <https://github.com/robotics-in-concert/rocon_concert/issues/302>`_
* comment out transition caused by wait for service timeout. closes `#302 <https://github.com/robotics-in-concert/rocon_concert/issues/302>`_
* fixing the logic to use remote controller slot in concert_client `#302 <https://github.com/robotics-in-concert/rocon_concert/issues/302>`_
* add state transition from busy to pending `#302 <https://github.com/robotics-in-concert/rocon_concert/issues/302>`_
* correcting the transition table
* add not in network node
* increae gap between nodes
* remove dummy from label
* add conductor state transition dotgraph generator for easy introspection closes `#301 <https://github.com/robotics-in-concert/rocon_concert/issues/301>`_
* add state transition graph
* configurable ros service timeout closes `#300 <https://github.com/robotics-in-concert/rocon_concert/issues/300>`_
* Contributors: Jihoon Lee

0.6.8 (2015-04-27)
------------------

0.6.7 (2015-04-06)
------------------
* add sleep for spining closes `#278 <https://github.com/robotics-in-concert/rocon_concert/issues/278>`_
* Contributors: Jihoon Lee

0.6.6 (2015-03-23)
------------------

0.6.5 (2015-02-27)
------------------

0.6.4 (2015-02-09)
------------------

0.6.3 (2015-01-05)
------------------

0.6.2 (2014-12-02)
------------------

0.6.1 (2014-11-21)
------------------
* [rocon_concert] permit esoteric names once more.
* Contributors: Daniel Stonier

0.6.0 (2014-08-25)
------------------
* documentation for the concert conductor.
* use proper lists for hubs/concerts now roslaunch can handle it.
* need the invite handle so we can uninvite it later, this was a regresion
  and this fixes `#230 <https://github.com/robotics-in-concert/rocon_concert/issues/230>`_.
* need the invite handle so we can uninvite it later, this was a regresion
  and this fixes `#230 <https://github.com/robotics-in-concert/rocon_concert/issues/230>`_.
* update publisher queue_size to avoid warning in indigo.
* pass ip info along in concert client publishing.
* remove debugging print.
* bifurcating shell/gui processing for concert_conductor_graph
* bugfix accidentally cancelled publisher pulls with the service pulls, `#222 <https://github.com/robotics-in-concert/rocon_concert/issues/222>`_
* remove unused conductor process pulls once they're done with, `#222 <https://github.com/robotics-in-concert/rocon_concert/issues/222>`_
* unregister the gateway info subscriber after we've finished with it.
* sphinx docs for concert conductor
* wireless handling and not using gone clients to generate concert aliases.
* small documentation on how the state machine works.
* Updated Idle->Available to reflect state name refactoring and relevant
  comments.
  Dummy transition handler for wireless dropouts.
* ConcertClientException no longer useed.
* eliminate redundant rapp_status and use uri's properly.
* bugfixes to reenable chatter concert.
* client status/gateway-info update handling and alot of minor fixes.
* concert client publishers reintegrated in for the new conductor
* don't go hunting for the gateway, configure it instead.
* concert transitions now working for conductor v2.
* relax timeouts on advertised concert client pulled services, fixes `#217 <https://github.com/robotics-in-concert/rocon_concert/issues/217>`_
* rapp_handler -> rocon_scheduler_requests
* a convenience rapp handler class for external users.
* expose invited clients only once their handles have been flipped across.
* interactions moved to rocon_tools, also updates for the rocon_utilities breakup.
* explanatory comment about the application namespace.
* make specific an exception message
* catch a shutdown exception.
* check rocon version of concert clients by conductor.
* updates for the new rocon uri
* platform tuple overhaul.
* update for recently moved modules to rocon_tools, also platform_tuples refactoring.
* remove unused variable.
* remove unused variable.
* cancel pulls when a client leaves, fixes `#169 <https://github.com/robotics-in-concert/rocon_concert/issues/169>`_ and ignore hash names, not friendly names, fixes `#170 <https://github.com/robotics-in-concert/rocon_concert/issues/170>`_.
* cancel_pulls -> _cancel_pulls, fixes `#165 <https://github.com/robotics-in-concert/rocon_concert/issues/165>`_
* better checking for local clients.
* bugfix service connection errors and delays from rogue clients in the conductor.
* republish uninvited clients, closes `#144 <https://github.com/robotics-in-concert/rocon_concert/issues/144>`_
* accept remote gateways with ip 'localhost' as local clients.
* working towards the compatibility tree scheduler.
* disambiguate concert client update topics
* expose connection statistics. closes `#35 <https://github.com/robotics-in-concert/rocon_concert/issues/35>`_
* only publish invited concert clients.
* handle corner case when uninviting disappearing clients.
* better handling and logging of failed invitations in various scenarios.
* only try to uninvite already invited clients.
* conductor cleanup.
* invite local clients only (good for testing), closes `#108 <https://github.com/robotics-in-concert/rocon_concert/issues/108>`_
* remove legacy web app client handling from the conductor, closes `#127 <https://github.com/robotics-in-concert/rocon_concert/issues/127>`_.
* catch a ros shutdown exception in the conductor spin.
* bugfix Invite->BatchInvite.
* external shutdown hooks for gateway and hub.
* added shutdown hook for the conductor, but it's not yet fully operational.
* bugfix a used variable before definition.
* service exception handler when inviting.
* quietly ignore invitation failures from local snobs.
* fix unhandled service exception when ros shutsdown.
* deprecate the old platform info message.
* refactoring for a concert master launcher and fix old legacy tutorials.
* scheduler
* parallel service working. some changes in conductor. conductor returns always false for lock now
* use wallsleeps to avoid simulation problems, closes `#46 <https://github.com/robotics-in-concert/rocon_concert/issues/46>`_.
* simple role manager launcher that publishes the concert icon, closes `#38 <https://github.com/robotics-in-concert/rocon_concert/issues/38>`_.
* concert_roles stub package.
* Contributors: Daniel Stonier, Jihoon Lee, piyushk

0.5.3 (2013-07-17)
------------------
* auto_invite is configurable with argument
* bugfix some holes in the human friendly naming

0.5.0 (2013-05-27 10:48)
------------------------
* include management of non-gateway clients (human clients).
* use human consumable aliases inside the concert instead of uuids directly.

0.3.0 (2013-02-05)
------------------
* better scanning for clients on the gateways.

0.2.0 (2013-02-01)
------------------
* upgrade to use the rocon gateways instead of complicated handshaking over xmlrpc/zeroconf

0.1.0 (2012-04-02)
------------------
* provide handles to the orchestration module for triggering start of a solution.
* interact over zeroconf and xmlrpc with rocon app managers.
