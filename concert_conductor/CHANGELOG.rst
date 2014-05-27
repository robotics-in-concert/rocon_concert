=========
Changelog
=========

Forthcoming
-----------
* drop management of non-gateway clients (moved to rocon interactions)
* gather and publish wireless connection statistics of wireless clients
* manage wireless client states (available, missing, gone).
* major code redesign to handle the complexity of wireless handling (formal state machine).

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
