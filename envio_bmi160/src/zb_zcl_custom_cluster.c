#include "zb_common.h"
#include "zb_zcl.h"
#include "zcl/zb_zcl_identify.h"
#include "zb_zdo.h"
#include "zb_aps.h"
#include "zb_zcl_custom_cluster.h"

zb_uint8_t gs_custom_server_received_commands[] =
{
  ZB_ZCL_CLUSTER_ID_CUSTOM_CLIENT_ROLE_GENERATED_CMD_LIST
};

zb_discover_cmd_list_t gs_custom_server_cmd_list =
{
  sizeof(gs_custom_server_received_commands), gs_custom_server_received_commands,
  0, NULL
};

zb_bool_t zb_zcl_process_custom_specific_commands_srv(zb_uint8_t param)
{
  if ( ZB_ZCL_GENERAL_GET_CMD_LISTS_PARAM == param )
  {
    ZCL_CTX().zb_zcl_cluster_cmd_list = &gs_custom_server_cmd_list;
    return ZB_TRUE;
  }
  return zb_zcl_process_on_off_specific_commands(param);
}

zb_ret_t check_value_custom_server(zb_uint16_t attr_id, zb_uint8_t endpoint, zb_uint8_t *value)
{
  ZVUNUSED(endpoint);
  ZVUNUSED(attr_id);

  switch (attr_id)
  {
    case ZB_ZCL_CUSTOM_CLUSTER_ATTR_CHAR_STRING_ID: 
    {
      return RET_OK;
    }
    default:
    {
      return RET_ERROR;
    }
  }
}

void zb_zcl_custom_attr_init_server()
{
  zb_zcl_add_cluster_handlers(ZB_ZCL_CLUSTER_ID_CUSTOM_ATTR,
                              ZB_ZCL_CLUSTER_SERVER_ROLE,
                              check_value_custom_server,
                              (zb_zcl_cluster_write_attr_hook_t)NULL,
                              zb_zcl_process_custom_specific_commands_srv);
}