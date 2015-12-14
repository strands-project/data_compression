^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package catkinized_libav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* After some hackery this works again for both devel and install builds
* Contributors: Nils Bore

0.0.9 (2015-02-25)
------------------
* removed the external libraries to be expected
* Revert "added new rosdep key as dependency. See https://github.com/strands-project/rosdistro/pull/370"
  This reverts commit 411841657371e342cb03df879a4b5d247be43151.
* added new rosdep key as dependency. See https://github.com/strands-project/rosdistro/pull/370
* Contributors: Marc Hanheide

0.0.8 (2014-11-23)
------------------

0.0.7 (2014-11-22)
------------------
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
