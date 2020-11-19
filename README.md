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

When working on the main freebsd branch every commit will be in the main freebsd
repository, this is how subtree works.
After doing a commit, to update the drm-subtree submodule do :
 - git subtree push --prefix sys/dev/drm/ drm-subtree master
And update again to have the latest changes :
 - git subtree pull --prefix sys/dev/drm/ drm-subtree master
Commit will appear twice in git log which is a bit weird so it might be better to commit
directly to this repository.

DRMKPI todos:
 - Remove struct task_struct and usage of td->td_lkpi_task
 - Finish checking that it doesn't conflict with linuxkpi
