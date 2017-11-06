importPackage(Packages.org.csstudio.opibuilder.scriptUtil);
 
var root = widget.getWidgetModel().getRootDisplayModel().getOpiFilePath().removeLastSegments(1);
var relative_path = "../diagOpi/scripts/synclibera"
var path = FileUtil.workspacePathToSysPath(root + "/" + relative_path);
 
ScriptUtil.executeSystemCommand("xterm -T 'sync BPMs' -e '" + path + " -Lsmf; read -p \\\"press return\\\"'", 120)
