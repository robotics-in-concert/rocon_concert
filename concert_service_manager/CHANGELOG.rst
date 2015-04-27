^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package concert_service_manager
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.8 (2015-04-27)
------------------
* improve log closes `#292 <https://github.com/robotics-in-concert/rocon_concert/issues/292>`_
* Contributors: Jihoon Lee

0.6.7 (2015-04-06)
------------------
* updates
* update doc for concert_service manager
* Contributors: Jihoon Lee

0.6.6 (2015-03-23)
------------------

0.6.5 (2015-02-27)
------------------
* add exception as param value
* update service instance to load parameter information from parameter_detail in service profile msg
* integrate cache manager with service pool
* fix typo
* do not write and read cache when disable_cache option is true and log clear
* update change a couple of rosparam tag into param tag
* discard updating service parameter when disabled_cache option is true
* update code document
* update to reflect service enabled status in cached file
* name change to disable cache and set the default value as true
* update load services from cache parameter
* addlaucher arguments regarding load from cache
* Contributors: dwlee

0.6.4 (2015-02-09)
------------------
* fixed typo in function name. also print actual error message to make debugging easier. closes `#266 <https://github.com/robotics-in-concert/rocon_concert/issues/266>`_
* do not write enabled field until `#258 <https://github.com/robotics-in-concert/rocon_concert/issues/258>`_ get resolved
* convert to string. KeyValue expects value as string
* now solution caches are created under solutions
* fix wrong try/catch
* adjust get rocon friendly name function
* move common util function and fix code as that
* add exception handling at some part
* share logic about load parameter from cahce, file, resource
* revert src
* add code documentation
* change name of auto enable service argument
* change modification checker name
* add extension name checker
* rename and code clear
* fix wrong argument
* add the string strip
* update service manager launch
* update
* update service manager and instance as adding service cache manager
* add function for loading parameter from cache
* create service profile cache manager
* function name change
* create utils.py and create cache maker regarding solution configuration
* Contributors: Jihoon Lee, Piyush Khandelwal, dwlee

0.6.3 (2015-01-05)
------------------

0.6.2 (2014-12-02)
------------------

0.6.1 (2014-11-21)
------------------
* now it properly prints not auto enabled service. `#247 <https://github.com/robotics-in-concert/rocon_concert/issues/247>`_
* add arg comments for all command. fix typo `#247 <https://github.com/robotics-in-concert/rocon_concert/issues/247>`_
* add all to enable all services
* now auto enable services are selectable
* Contributors: Jihoon Lee

0.6.0 (2014-08-25)
------------------
* update publisher queue_size to avoid warning in indigo.
* temporary code for upgraded package exports indexer.
* usable configuration for service priorities.
* fixes to allow concert master to run without service configuration, fixes `#215 <https://github.com/robotics-in-concert/rocon_concert/issues/215>`_
* rocon_service -> concert_service
* reloading of solution configuration and service profiles as needed.
* remove the enabled flag from instances.
* service profiles loading up ok with the new framework.
* middle of service configuration update.
* solution configuration validation and tests.
* solution configuration for services.
* slots added for clarity.
* load parameters before starting the services, fixes `#205 <https://github.com/robotics-in-concert/rocon_concert/issues/205>`_.
* hint with parameters extension when looking for parameters files.
* pep8 and spelling fixes, `#200 <https://github.com/robotics-in-concert/rocon_concert/issues/200>`_.
* bugfix handling of service profile loading when a service is incorrect, `#198 <https://github.com/robotics-in-concert/rocon_concert/issues/198>`_, `#199 <https://github.com/robotics-in-concert/rocon_concert/issues/199>`_
* rename resource to resource_name
* now services are loading solution configuration properly
* function name updates
* dynamic solution configuration apply works
* concert service uses resource instead of dispaly name
* reloading solution configuration in the middle
* interactions moved to rocon_tools, also updates for the rocon_utilities breakup.
* erge parameter updates with interactions overhaul.
* removing func pointer
* typo fix
* fix logic to not use function pointer
* pep8
* parameter support ready
* provide warning when services are not found on the package path.
* interactions upgrades for master, service manager and chatter.
* parse icon only if yaml has icon
* icon parse
* support for shadow service types.
* implement service exports instead of service lists, closes `#175 <https://github.com/robotics-in-concert/rocon_concert/issues/175>`_
* concert_service_roslaunch -> concert_service_link_graph.
* saner list services topic name.
* EnableConcertService.srv -> EnableService.srv updates.
* concert_roles -> rocon_interactions
* detect client changes in the scheduler.
* enable/disable services with new scheduler working, also debugging scheduler requests script.
* demo scheduler/requester enable/disable services back in.
* infrastructure for releasing resources on the requester side.
* fix mutex deadlock issue in service manager
* update to use jack's requester.
* conductor cleanup.
* concert_utilities package prepared, `#80 <https://github.com/robotics-in-concert/rocon_concert/issues/80>`_
* pep8, slots refactoring, also added locks to enabling/disabling services, `#91 <https://github.com/robotics-in-concert/rocon_concert/issues/91>`_
* use the rocon screen parameter instead of blindly forcing it, refs `#91 <https://github.com/robotics-in-concert/rocon_concert/issues/91>`_.
* remove complicated threading from the service instance monitoring, refs `#91 <https://github.com/robotics-in-concert/rocon_concert/issues/91>`_.
* external shutdown hooks for gateway and hub.
* rosdep fix for rospkg->python-rospkg
* added shutdown hook for the conductor, but it's not yet fully operational.
* unload resources when disabling services.
* rostime
* sleep to wallsleep
* wasn't pusing services into the services namespace, `#109 <https://github.com/robotics-in-concert/rocon_concert/issues/109>`_
* parameters loading into the service namespace, `#80 <https://github.com/robotics-in-concert/rocon_concert/issues/80>`_.
* cleaning up some logging output.
* warning if service fails to be enabled, `#96 <https://github.com/robotics-in-concert/rocon_concert/issues/96>`_.
* use wallsleep, not sleep, `#103 <https://github.com/robotics-in-concert/rocon_concert/issues/103>`_.
* service names should be valid rosgraph names.
* fixes parsing launch file
* rename fix for concert_service_roslaunch
* log data to description
* merging new changes
* mid autostarting services.
* renaming logger name `#85 <https://github.com/robotics-in-concert/rocon_concert/issues/85>`_
* adding logwarn
* merging with hydro-devel
* merging interactions with latest service changes
* service to remocon pipeline looking good.
* some cleaning
* introducing launcher type. roslaunch and custom
* turtle launcher. enable working version of static link graph. disable no work yet
* transferred concert_solution to rocon_tutorials/concert_tutorial.
* trivial pep8 fixes.
* service runs independently now. changes in service description
* Merge branch 'hydro-devel' into enble_fix
* terminal titles for solution concert.
* refactoring, we use '_''s by convention in rocon, also pep8.
* Contributors: Daniel Stonier, Jihoon Lee

* update publisher queue_size to avoid warning in indigo.
* temporary code for upgraded package exports indexer.
* usable configuration for service priorities.
* fixes to allow concert master to run without service configuration, fixes `#215 <https://github.com/robotics-in-concert/rocon_concert/issues/215>`_
* rocon_service -> concert_service
* reloading of solution configuration and service profiles as needed.
* remove the enabled flag from instances.
* service profiles loading up ok with the new framework.
* middle of service configuration update.
* solution configuration validation and tests.
* solution configuration for services.
* slots added for clarity.
* load parameters before starting the services, fixes `#205 <https://github.com/robotics-in-concert/rocon_concert/issues/205>`_.
* hint with parameters extension when looking for parameters files.
* pep8 and spelling fixes, `#200 <https://github.com/robotics-in-concert/rocon_concert/issues/200>`_.
* bugfix handling of service profile loading when a service is incorrect, `#198 <https://github.com/robotics-in-concert/rocon_concert/issues/198>`_, `#199 <https://github.com/robotics-in-concert/rocon_concert/issues/199>`_
* rename resource to resource_name
* now services are loading solution configuration properly
* function name updates
* dynamic solution configuration apply works
* concert service uses resource instead of dispaly name
* reloading solution configuration in the middle
* interactions moved to rocon_tools, also updates for the rocon_utilities breakup.
* erge parameter updates with interactions overhaul.
* removing func pointer
* typo fix
* fix logic to not use function pointer
* pep8
* parameter support ready
* provide warning when services are not found on the package path.
* interactions upgrades for master, service manager and chatter.
* parse icon only if yaml has icon
* icon parse
* support for shadow service types.
* implement service exports instead of service lists, closes `#175 <https://github.com/robotics-in-concert/rocon_concert/issues/175>`_
* concert_service_roslaunch -> concert_service_link_graph.
* saner list services topic name.
* EnableConcertService.srv -> EnableService.srv updates.
* concert_roles -> rocon_interactions
* detect client changes in the scheduler.
* enable/disable services with new scheduler working, also debugging scheduler requests script.
* demo scheduler/requester enable/disable services back in.
* infrastructure for releasing resources on the requester side.
* fix mutex deadlock issue in service manager
* update to use jack's requester.
* conductor cleanup.
* concert_utilities package prepared, `#80 <https://github.com/robotics-in-concert/rocon_concert/issues/80>`_
* pep8, slots refactoring, also added locks to enabling/disabling services, `#91 <https://github.com/robotics-in-concert/rocon_concert/issues/91>`_
* use the rocon screen parameter instead of blindly forcing it, refs `#91 <https://github.com/robotics-in-concert/rocon_concert/issues/91>`_.
* remove complicated threading from the service instance monitoring, refs `#91 <https://github.com/robotics-in-concert/rocon_concert/issues/91>`_.
* external shutdown hooks for gateway and hub.
* rosdep fix for rospkg->python-rospkg
* added shutdown hook for the conductor, but it's not yet fully operational.
* unload resources when disabling services.
* rostime
* sleep to wallsleep
* wasn't pusing services into the services namespace, `#109 <https://github.com/robotics-in-concert/rocon_concert/issues/109>`_
* parameters loading into the service namespace, `#80 <https://github.com/robotics-in-concert/rocon_concert/issues/80>`_.
* cleaning up some logging output.
* warning if service fails to be enabled, `#96 <https://github.com/robotics-in-concert/rocon_concert/issues/96>`_.
* use wallsleep, not sleep, `#103 <https://github.com/robotics-in-concert/rocon_concert/issues/103>`_.
* service names should be valid rosgraph names.
* fixes parsing launch file
* rename fix for concert_service_roslaunch
* log data to description
* merging new changes
* mid autostarting services.
* renaming logger name `#85 <https://github.com/robotics-in-concert/rocon_concert/issues/85>`_
* adding logwarn
* merging with hydro-devel
* merging interactions with latest service changes
* service to remocon pipeline looking good.
* some cleaning
* introducing launcher type. roslaunch and custom
* turtle launcher. enable working version of static link graph. disable no work yet
* transferred concert_solution to rocon_tutorials/concert_tutorial.
* trivial pep8 fixes.
* service runs independently now. changes in service description
* Merge branch 'hydro-devel' into enble_fix
* terminal titles for solution concert.
* refactoring, we use '_''s by convention in rocon, also pep8.
* Contributors: Daniel Stonier, Jihoon Lee

0.5.5 (2013-08-30)
------------------

0.5.4 (2013-07-19)
------------------

0.5.3 (2013-07-17)
------------------

0.5.2 (2013-06-10)
------------------

0.5.1 (2013-05-27 11:46)
------------------------

0.5.0 (2013-05-27 10:48)
------------------------

0.3.0 (2013-02-05)
------------------

0.2.0 (2013-02-01)
------------------

0.1.1 (2012-12-12)
------------------

0.1.0 (2012-04-02)
------------------
