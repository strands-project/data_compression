^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package catkinized_libav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Got it working by adding the libraries as catkin package exports
* Contributors: Nils Bore

0.0.6 (2014-11-21)
------------------

0.0.5 (2014-11-21)
------------------

0.0.4 (2014-11-21)
------------------

0.0.3 (2014-11-19)
------------------

0.0.1 (2014-11-14)
------------------
* Merge commit 'e3b1bba6165b7e6e75b9ae3b4c58ffb9a94324e0' as 'catkinized_libav/libav_trunk'
* Removed libav submodule
* Corrected maintainer's in packages
* Disabled x264 support in libav since it can mess up compilation and we don't need it for depth image transfer
* Cleaned up the catkinized_libav cmake file a bit
* Added install targets for catkinized_libav
* Trying to add the install targets correctly, still need to copy the symlinks
* Changed the cmake to work with submodules instead of cloning
* Change the branch of libav to 9.8
* Added libav as a submodule
* Added the necessary build depends in package.xml
* Added the catkinized libav package
* Contributors: Nils Bore
