from AppKit import NSWorkspace
from Quartz import kCGWindowListOptionOnScreenOnly, kCGNullWindowID, CGWindowListCopyWindowInfo

def find_window_geometry_by_name (appName):
    workspace = NSWorkspace.sharedWorkspace()
    runningApps = workspace.runningApplications()
    for app in runningApps:
        #if app.isActive():
        if app.localizedName() == appName:
            #print ('one more', app.localizedName())
            options = kCGWindowListOptionOnScreenOnly
            windowList = CGWindowListCopyWindowInfo(options,
                                                kCGNullWindowID)
            for window in windowList:
                if window['kCGWindowOwnerName'] == app.localizedName():
                    #print(window.getKeys_)
                    kCGWindowBounds = window['kCGWindowBounds']
                    #print(kCGWindowBounds)
                    # height = kCGWindowBounds['Height']
                    # width = kCGWindowBounds['Width']
                    # X = kCGWindowBounds['X']
                    # Y = kCGWindowBounds['Y']
                    return kCGWindowBounds #(X, Y, height, width)
                    break
            break