#include "vicon_client/vicon_client.h"
#include "vicon_client/vicon_ihm.h"

BEGIN_VICON_HANDLER_TABLE
	VICON_HANDLER_TABLE_ENTRY(vicon_file_init, vicon_file_process, vicon_file_release, NULL)
	VICON_HANDLER_TABLE_ENTRY(vicon_ihm_init, vicon_ihm_process, vicon_ihm_release, NULL)
END_VICON_HANDLER_TABLE
