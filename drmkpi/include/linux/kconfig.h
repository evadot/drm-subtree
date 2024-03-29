/* Public Domain */

#ifndef __DRMCOMPAT_LINUX_KCONFIG_H__
#define	__DRMCOMPAT_LINUX_KCONFIG_H__

/*
 * Checking if an option is defined would be easy if we could do CPP inside CPP.
 * The defined case whether -Dxxx or -Dxxx=1 are easy to deal with.  In either
 * case the defined value is "1". A more general -Dxxx=<c> case will require
 * more effort to deal with all possible "true" values. Hope we do not have
 * to do this as well.
 * The real problem is the undefined case.  To avoid this problem we do the
 * concat/varargs trick: "yyy" ## xxx can make two arguments if xxx is "1"
 * by having a #define for yyy_1 which is "ignore,".
 * Otherwise we will just get "yyy".
 * Need to be careful about variable substitutions in macros though.
 * This way we make a (true, false) problem a (don't care, true, false) or a
 * (don't care true, false).  Then we can use a variadic macro to only select
 * the always well known and defined argument #2.  And that seems to be
 * exactly what we need.  Use 1 for true and 0 for false to also allow
 * #if IS_*() checks pre-compiler checks which do not like #if true.
 */
#define ___XAB_1		dontcare,
#define ___IS_XAB(_ignore, _x, ...)	(_x)
#define	__IS_XAB(_x)		___IS_XAB(_x 1, 0)
#define	_IS_XAB(_x)		__IS_XAB(__CONCAT(___XAB_, _x))

/* This is if CONFIG_ccc=y. */
#define	IS_BUILTIN(_x)		_IS_XAB(_x)
/* This is if CONFIG_ccc=m. */
#define	IS_MODULE(_x)		_IS_XAB(_x ## _MODULE)
/* This is if CONFIG_ccc is compiled in(=y) or a module(=m). */
#define	IS_ENABLED(_x)		(IS_BUILTIN(_x) || IS_MODULE(_x))
/*
 * This is weird case.  If the CONFIG_ccc is builtin (=y) this returns true;
 * or if the CONFIG_ccc is a module (=m) and the caller is built as a module
 * (-DMODULE defined) this returns true, but if the callers is not a module
 * (-DMODULE not defined, which means caller is BUILTIN) then it returns
 * false.  In other words, a module can reach the kernel, a module can reach
 * a module, but the kernel cannot reach a module, and code never compiled
 * cannot be reached either.
 * XXX -- I'd hope the module-to-module case would be handled by a proper
 * module dependency definition (MODULE_DEPEND() in FreeBSD).
 */
#define	IS_REACHABLE(_x)	(IS_BUILTIN(_x) || \
				    (IS_MODULE(_x) && IS_BUILTIN(MODULE)))

#endif /* __DRMCOMPAT_LINUX_KCONFIG_H__ */
