^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package libav_image_transport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* added catkinized_libav as dependency to make sure it's built prior to libav_image_transport
* Contributors: Marc Hanheide

0.0.8 (2014-11-23)
------------------

0.0.7 (2014-11-22)
------------------
* Got it working by adding the libraries as catkin package exports
* Contributors: Nils Bore

0.0.6 (2014-11-21)
------------------
* Revert "Try to link all the libraries explicitly to see if that works"
* Contributors: Nils Bore

0.0.5 (2014-11-21)
------------------
* Tried to link all the libraries explicitly instead
* Contributors: Nils Bore

0.0.4 (2014-11-21)
------------------
* Commented hihde library symbols
* Contributors: Nils Bore

0.0.3 (2014-11-19)
------------------

0.0.1 (2014-11-14)
------------------
* Update package.xml
* Corrected maintainer's in packages
* Added install targets for catkinized_libav
* Moved the README into correct package
* Now just using the original headers because why not
* Added the frame_id in the message of the encoded frame Package msg so we can use multiple cameras
* Removed padding to the right of the image and hacked the depth frame in the header for now, everything working
* Got the depth_registered cloud running at least
* Changed the dynamic config files to have depth image compression with ffv1 as default
* Removed the libav cmake module as it's not needed with the catkinized version
* Made the package depend on catkinized_libav instead of the normal library
* add missing dependency
* fix pixel formats
* add dynamic reconfigure server on subscriber side
* fix cmake include
* fix pts implementation
* add compile flag
* fix error with key_frame flag
* fix errors occurring in destructors
* change dependency classification
* fix timestamp implementation
* fix bug with packet
* leave the seach paths to cmake
* change dependency
* fix bug with backport
* fix problem with new version format of libav
* add install directives
* Update libav_plugins.xml
* initial commit
* Contributors: Dominique Hunziker, Marc Hanheide, Nils Bore, dominiquehunziker
