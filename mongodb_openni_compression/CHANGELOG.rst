^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mongodb_openni_compression
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2015-12-14)
------------------

0.1.1 (2015-12-14)
------------------

0.1.0 (2015-12-14)
------------------

0.0.11 (2015-12-14)
-------------------

0.0.10 (2015-08-19)
-------------------

0.0.9 (2015-02-25)
------------------
* fixed the mongodb_play.py call
  The name of the RGB topic, under which it is stored in mongo, was wrong
* Contributors: Vojtech Novak

0.0.8 (2014-11-23)
------------------
* Changed the topic of the recorded streams as well
* Added the option to change the compression type, switched default rgb to normal compressed
* Contributors: Nils Bore

0.0.7 (2014-11-22)
------------------

0.0.6 (2014-11-21)
------------------

0.0.5 (2014-11-21)
------------------

0.0.4 (2014-11-21)
------------------
* Filled in the README
* Added a README file
* Changed the name of the server class
* Contributors: Nils Bore

0.0.3 (2014-11-19)
------------------
* Covered all the cases with the action server and changed the names of the recorded topics to include the camera name
* Contributors: Nils Bore

0.0.2 (2014-11-18)
------------------
* made all version numbers the same across the repo
* Added a launch file for launching everything together
* Added roslaunch axserver as run depend
* Added proper install targets for mongo and rosbag compression
* Added an action server for recording openni camera topics on demand
* Initial version of mongodb logger
* Contributors: Marc Hanheide, Nils Bore

0.0.1 (2014-11-14)
------------------
