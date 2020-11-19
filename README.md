How to use :

From a checkout freebsd git repo :
- git checkout -b drm-base-subtree
- git remote add drm-subtree https://github.com/evadot/drm-subtree.git
- git subtree add --prefix sys/dev/drm/ drm-subtree master
- git am sys/dev/drm/*

To update:
 - git fetch drm-subtree
 - git subtree pull --prefix sys/dev/drm/ drm-subtree master
 - Check if there is any new patches in extra_patches and git am them
