^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package concert_conductor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.5 (2013-08-30)
------------------

0.5.4 (2013-07-19)
------------------

0.5.3 (2013-07-17)
------------------
* No params dir to install
* auto_invite is configurable with argument
* bugfix renamed app list variable from pairing merge.
* bugfix some holes in the human friendly naming, make sure we go back and plug missing holes.

0.5.2 (2013-06-10)
------------------
* 0.5.1
* install launch directory in conductor
* 0.5.0

0.5.1 (2013-05-27 11:46)
------------------------
* install launch directory in conductor

0.5.0 (2013-05-27 10:48)
------------------------
* update for non-gateway clients.
* concert master launchers moved to conductor.
* local concert name fixes.
* trivial debugging comments
* concert conductor send invite with local name
* handle uuid's by using a human consumable name, also remove the leading slash that was in the human readable names before.
* 0.4.0
* gateway info now a msg.

0.3.0 (2013-02-05)
------------------
* Merge branch 'groovy-devel' of https://github.com/robotics-in-concert/rocon_concert into groovy-devel
* updating jihoon e-mail
* taking the concert client out of the loop
* scanning for clients on the gateways.
* concert status -> app manager status, part of first redesign.

0.2.0 (2013-02-01)
------------------
* pulling list apps, also correcting advertising behaviour in the client.
* cleaning, don't use unnecessary params.
* fleshed put platform info retrieval.
* platform info to rocon_app_manager_msgs
* rocon_orchestration->rocon_concert
* refactoring app->rapp.
* catkinization fixes, but still not yet working.
* catkinized.
* no more solution handling.
* started adding a stop, but aborted for now. also bugfixed in place with
  a wait for service.
* start solution, also pass on remappings.
* orchestration plugged in.
* reinstituted app list in concert client information.
* Don't latch yet till rosbridge fixes itself.
* less spammy concert client list updates.
* safely check platform info services.
* publishing concert client information.
* rewire conductor to send the concert name in the invitation, lost when auto-invite service was cancelled.
* revert to old topic name.
* minor fixes.
* minor fixes.
* handle bad clients.
* update licenses.
* importing from rocon_app_platform.

0.1.1 (2012-12-12)
------------------

0.1.0 (2012-04-02)
------------------
