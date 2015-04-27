Changelog
=========

0.6.8 (2015-04-27)
------------------
* add concert_software_farmer is missing dependency clsoses `#289 <https://github.com/robotics-in-concert/rocon_concert/issues/289>`_
* Contributors: Jihoon Lee

0.6.7 (2015-04-06)
------------------

0.6.6 (2015-03-23)
------------------

0.6.5 (2015-02-27)
------------------
* more doc strings
* create doc string for roslaunch
* disable zerocon option available
* Contributors: Jihoon Lee

0.6.4 (2015-02-09)
------------------
* add graph to stringpublihser closes `#270 <https://github.com/robotics-in-concert/rocon_concert/issues/270>`_
* Merge branch 'indigo' into admin_app_patch
* rename software_manager as software_farmer
* change name of auto enable service argument
* add the concert_name arg in service manager launcher
* Merge branch 'indigo' into software_farm
* draft of software manager
* Contributors: Jihoon Lee, dwlee

0.6.3 (2015-01-05)
------------------

0.6.2 (2014-12-02)
------------------
* add webserver_address param resolves `#251 <https://github.com/robotics-in-concert/rocon_concert/issues/251>`_
* Contributors: Jihoon Lee

0.6.1 (2014-11-21)
------------------
* add option to start rosbridge with concert.
* add arg comments for all command. fix typo `#247 <https://github.com/robotics-in-concert/rocon_concert/issues/247>`_
* now auto enable services are selectable
* Contributors: Jihoon Lee

0.6.0 (2014-08-25)
------------------
* documentation for the concert conductor.
* documentation for the concert conductor.
* documentation for concert master.
* use proper lists for hubs/concerts now roslaunch can handle it.
* enable preemption as an option in the compatibility tree scheduler.
* fixed typo in concert master launch. services were getting enabled by default irrepective of parameter
* expose gateway_unavailable_timeout as an arg
* simplify concert info script.
* updating concert info to sync with rocon master info
* rocon_service -> concert_service
* Merge branch 'hydro-devel' into indigo
* oops used arg instead of param
* rosbridge parameter passing
* integrating jack's scheduler.
* minor comment updates.
* scheduler launchers.
* new more flexible scheduler arg speciification and defaults.
* doc update
* fix install rule for concert master script.
* middle of service configuration update.
* fix install rule for concert master script.
* Merge branch 'hydro-devel' into parameter_overriding
* shadow type added
* doc tyop
* rename name to resource
* now depending on rocon_icons for its own icons.
* merging hydro-devel
* Merge branch 'hydro-devel' into parameter_overriding
* concert info now going via rocon_master_info.
* solution service configuration added
* description updae
* service argument now serves service list resource location
* interactions moved to rocon_tools, also updates for the rocon_utilities breakup.
* update service doc
* update service document
* remove last vestiges of the demo scheduler.
* typo fixes
* init doc
* interactions upgrades for master, service manager and chatter.
* pass command line args through the concert_info scripot.
* implement service exports instead of service lists, closes `#175 <https://github.com/robotics-in-concert/rocon_concert/issues/175>`_
* concert version in concert info.
* update for recently moved modules to rocon_tools, also platform_tuples refactoring.
* concert_roles -> rocon_interactions
* remove legacy references to concert_orchestra.
* use an environment variable for setting the gateway network interface on
  multiple interface machines.
* rename dependency on concert_schedulers correctly, closes `#149 <https://github.com/robotics-in-concert/rocon_concert/issues/149>`_
* gateway network interface.
* working towards the compatibility tree scheduler.
* exposed args should use defaults, not values
* Set up an arg to transfer a scheduler type through.
* Remove legacy master.
* No more dependency on concert_service
  All has gone to the concert_service_manager.
* scheduler -> scheduler.py
* a pirate launcher for testing.
* configure the hub arguments in the concert master launcher.
* invite local clients only (good for testing), closes `#108 <https://github.com/robotics-in-concert/rocon_concert/issues/108>`_
* external shutdown hooks for gateway and hub.
* revert role manager loader - not needed.
* reverting roles_and apps argument
* adding concert_roles_apps arguement in concertmaster
* mid autostarting services.
* service to remocon pipeline looking good.
* Chang run_depend(concert_manager to concert_service_manager)
* transferred concert_solution to rocon_tutorials/concert_tutorial.
* concert master dependencies set correctly.
* refactoring, we use '_''s by convention in rocon, also pep8.
* refactoring for a concert master launcher and fix old legacy tutorials.
* iterating on role manager, `#47 <https://github.com/robotics-in-concert/rocon_concert/issues/47>`_, `#51 <https://github.com/robotics-in-concert/rocon_concert/issues/51>`_.
* iterating on role manager, `#47 <https://github.com/robotics-in-concert/rocon_concert/issues/47>`_, `#51 <https://github.com/robotics-in-concert/rocon_concert/issues/51>`_.
* concert master publishing concert information, removed from role_manager.
* Contributors: Daniel Stonier, Jihoon Lee, Piyush Khandelwal
