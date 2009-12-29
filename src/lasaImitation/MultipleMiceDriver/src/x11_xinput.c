/*
 * Support for the X11 XInput extension.
 *
 * Please see the file LICENSE.txt in the source's root directory.
 *
 *  This file written by Ryan C. Gordon.
 */

#include "manymouse.h"

/* Try to use this on everything but Windows and Mac OS by default... */
#ifndef SUPPORT_XINPUT
#if ( (defined(_WIN32) || defined(__CYGWIN__)) )
#define SUPPORT_XINPUT 0
#elif ( (defined(__MACH__)) && (defined(__APPLE__)) )
#define SUPPORT_XINPUT 0
#else
#define SUPPORT_XINPUT 1
#endif
#endif

#if SUPPORT_XINPUT

//#error this code is incomplete. Do not use unless you are fixing it.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dlfcn.h>
#include <X11/extensions/XInput.h>

/* 32 is good enough for now. */
#define MAX_MICE 32
typedef struct
{
    XDevice *device;
    int min_x;
    int min_y;
    int max_x;
    int max_y;
    char name[64];
} MouseStruct;

static MouseStruct mice[MAX_MICE];
static unsigned int available_mice = 0;

static Display *display = NULL;
static XExtensionVersion *extver = NULL;
static XDeviceInfo *device_list = NULL;
static int device_count = 0;

/*
 * You _probably_ have Xlib on your system if you're on a Unix box where you
 *  are planning to plug in multiple mice. That being said, we don't want
 *  to force a project to add Xlib to their builds, or force the end-user to
 *  have Xlib installed if they are otherwise running a console app that the
 *  evdev driver would handle.
 *
 * We load all Xlib symbols at runtime, and fail gracefully if they aren't
 *  available for some reason...ManyMouse might be able to use the evdev
 *  driver or at least return a zero.
 *
 * On Linux (and probably others), you'll need to add -ldl to your link line,
 *  but it's part of glibc, so this is pretty much going to be there.
 */

static void *libx11 = NULL;
static void *libxext = NULL;
static void *libxi = NULL;
typedef int (*XExtErrHandler)(Display *, _Xconst char *, _Xconst char *);
static XExtErrHandler (*pXSetExtensionErrorHandler)(XExtErrHandler h) = 0;
static Display* (*pXOpenDisplay)(_Xconst char*) = 0;
static int (*pXCloseDisplay)(Display*) = 0;
static int (*pXFree)(void*) = 0;
static XExtensionVersion* (*pXGetExtensionVersion)(Display*,_Xconst char*) = 0;
static XDeviceInfo* (*pXListInputDevices)(Display*,int*) = 0;
static void	(*pXFreeDeviceList)(XDeviceInfo*) = 0;
static XDevice* (*pXOpenDevice)(Display*,XID) = 0;
static int (*pXCloseDevice)(Display*,XDevice*) = 0;

static int symlookup(void *dll, void **addr, const char *sym)
{
    *addr = dlsym(dll, sym);
    if (*addr == NULL)
        return(0);

    return(1);
} /* symlookup */

static int find_api_symbols(void)
{
    void *dll = NULL;

    #define LOOKUP(x) { if (!symlookup(dll, (void **) &p##x, #x)) return(0); }
    dll = libx11 = dlopen("libX11.so.6", RTLD_GLOBAL | RTLD_LAZY);
    if (dll == NULL)
        return(0);

    LOOKUP(XOpenDisplay);
    LOOKUP(XCloseDisplay);
    LOOKUP(XFree);

    dll = libxext = dlopen("libXext.so.6", RTLD_GLOBAL | RTLD_LAZY);
    if (dll == NULL)
        return(0);

    LOOKUP(XSetExtensionErrorHandler);

    dll = libxi = dlopen("libXi.so.6", RTLD_GLOBAL | RTLD_LAZY);
    if (dll == NULL)
        return(0);

    LOOKUP(XGetExtensionVersion);
    LOOKUP(XListInputDevices);
    LOOKUP(XFreeDeviceList);
    LOOKUP(XOpenDevice);
    LOOKUP(XCloseDevice);

    #undef LOOKUP

    return(1);
} /* find_api_symbols */


static void xinput_cleanup(void)
{
    int i;

    if (display != NULL)
    {
        for (i = 0; i < available_mice; i++)
        {
            if (mice[i].device)
                pXCloseDevice(display, mice[i].device);
        } /* for */
    } /* if */

    if (extver != NULL)
    {
        pXFree(extver);
        extver = NULL;
    } /* if */

    if (device_list != NULL)
    {
        pXFreeDeviceList(device_list);
        device_list = NULL;
    } /* if */

    if (display != NULL)
    {
        pXCloseDisplay(display);
        display = NULL;
    } /* if */

    memset(mice, '\0', sizeof (mice));
    available_mice = 0;

    #define LIBCLOSE(lib) { if (lib != NULL) { dlclose(lib); lib = NULL; } }
    LIBCLOSE(libxi);
    LIBCLOSE(libxext);
    LIBCLOSE(libx11);
    #undef LIBCLOSE
} /* xinput_cleanup */


/* Just in case this is compiled as a C++ module... */
static XID get_x11_any_class(const XAnyClassPtr anyclass)
{
#if defined(__cplusplus) || defined(c_plusplus)
    return anyclass->c_class;
#else
    return anyclass->class;
#endif
} /* get_x11_any_class */


static int init_mouse(MouseStruct *mouse, const XDeviceInfo *devinfo)
{
    int i;
    int has_axes = 0;
    int has_buttons = 0;
    XAnyClassPtr any = devinfo->inputclassinfo;

    if (devinfo->use == IsXPointer)
        return(0);  /* sucks! Can't open a mouse that is the system pointer! */

    if (devinfo->use == IsXKeyboard)
        return(0);  /* definitely not a mouse. :) */

    for (i = 0; i < devinfo->num_classes; i++)
    {
        XID cls = get_x11_any_class(any);

        if (cls == KeyClass)
            return(0);  /* a keyboard? */

        else if (cls == ButtonClass)
        {
            const XButtonInfo *info = (const XButtonInfo *) any;
            if (info->num_buttons > 0)
                has_buttons = 1;
        } /* else if */

        else if (cls == ValuatorClass)
        {
            const XValuatorInfo *info = (const XValuatorInfo *) any;
            if (info->num_axes != 2)  /* joystick? */  /* !!! FIXME: this isn't right! */
                return 0;

            has_axes = 1;
            mouse->min_x = info->axes[0].min_value;
            mouse->max_x = info->axes[0].max_value;
            mouse->min_y = info->axes[1].min_value;
            mouse->max_y = info->axes[1].max_value;
        } /* else if */

        any = (XAnyClassPtr) ((char *) any + any->length);
    } /* for */

    if ((!has_axes) || (!has_buttons))
        return(0);  /* probably not a mouse. */

    mouse->device = pXOpenDevice(display, devinfo->id);
    if (mouse->device == NULL)
        return(0);

    strncpy(mouse->name, devinfo->name, sizeof (mouse->name));
    mouse->name[sizeof (mouse->name) - 1] = '\0';
    return(1);
} /* init_mouse */


static int (*Xext_handler)(Display *, _Xconst char *, _Xconst char *) = NULL;
static int xext_errhandler(Display *d, _Xconst char *ext, _Xconst char *reason)
{
    /* Don't do anything (write an error to stderr) if extension is missing */
	if (strcmp(reason, "missing") == 0)
		return 0;

	return Xext_handler(d, ext, reason);
}


static int x11_xinput_init_internal(void)
{
    int i;

    xinput_cleanup();  /* just in case... */

    if (getenv("MANYMOUSE_NO_XINPUT") != NULL)
        return(-1);

    if (!find_api_symbols())
        return(-1);  /* couldn't find all needed symbols. */

    display = pXOpenDisplay(NULL);
    if (display == NULL)
        return(-1);  /* no X server at all */

    /* Stop stderr output in case XInput extension is missing... */
    Xext_handler = pXSetExtensionErrorHandler(xext_errhandler);
    extver = pXGetExtensionVersion(display, INAME);
    pXSetExtensionErrorHandler(Xext_handler);
    Xext_handler = NULL;

    if ((extver == NULL) || (extver == (XExtensionVersion *) NoSuchExtension))
        return(-1);  /* no such extension */

    if (extver->present == XI_Absent)
        return(-1);  /* extension not available. */

    device_list = pXListInputDevices(display, &device_count);
    if (device_list == NULL)
        return(-1);

    printf("dc %d\n",device_count);
    for (i = 0; i < device_count; i++)
    {
        MouseStruct *mouse = &mice[available_mice];
        if (init_mouse(mouse, &device_list[i]))
            available_mice++;
    } /* for */

    return(available_mice);
} /* x11_xinput_init_internal */


static int x11_xinput_init(void)
{
    int retval = x11_xinput_init_internal();
    if (retval < 0)
        xinput_cleanup();
    return(retval);
} /* x11_xinput_init */


static void x11_xinput_quit(void)
{
    xinput_cleanup();
} /* x11_xinput_quit */


static const char *x11_xinput_name(unsigned int index)
{
    if (index < available_mice)
        return(mice[index].name);
    return(NULL);
} /* x11_xinput_name */


static int x11_xinput_poll(ManyMouseEvent *event)
{
    return(0);  /* !!! FIXME */
} /* x11_xinput_poll */

static const ManyMouseDriver ManyMouseDriver_interface =
{
    x11_xinput_init,
    x11_xinput_quit,
    x11_xinput_name,
    x11_xinput_poll
};

const ManyMouseDriver *ManyMouseDriver_xinput = &ManyMouseDriver_interface;

#else
const ManyMouseDriver *ManyMouseDriver_xinput = 0;
#endif /* SUPPORT_XINPUT blocker */

/* end of x11_xinput.c ... */

