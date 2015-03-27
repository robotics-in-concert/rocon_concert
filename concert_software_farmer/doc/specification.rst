Software Specification
======================

Software are specified in yaml to be loaded by Concert Softwarer Farmer. 
It is designed to dynamically start/stop sharable software(or ros component) for a multiple higher level controllers with configurable parameters.
It allows a multiple of Concert Service Instances to share common applications like World Canvas Server, Web Video Server.


Software Types
--------------

 * roslaunch

Yaml Specification
------------------

A single software specification in yaml has the following format:

.. code-block:: yaml

  name:
  description:
  author:
  launch:
  max_count:
  parameters:
    - name:
      value:
