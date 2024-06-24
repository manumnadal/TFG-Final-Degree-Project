
#include <zboss_api.h>
#include <zboss_api_addons.h>
//#include <zb_mem_config_med.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
#include <zigbee/zigbee_zcl_scenes.h>
#include <zb_nrf_platform.h>
#include <zcl/zb_zcl_custom_cluster.h>

LOG_MODULE_REGISTER(zcl_custom, LOG_LEVEL_INF);

static void custom_cmd_handler(zb_uint8_t param, zb_zcl_custom_addr_t* addr) {
    zb_zcl_parse_status_t status;

    TRACE_MSG(TRACE_ZCL1, "> custom_cmd_handler param %i", (FMT__H, param));

    if (status == ZB_ZCL_PARSE_STATUS_SUCCESS) {
        //LOG_INF("Received");
    }

    TRACE_MSG(TRACE_ZCL1, "< custom_cmd_handler", (FMT__0));
}

zb_bool_t  zb_zcl_process_custom_specific_commands(zb_uint8_t param)
{
  zb_zcl_attr_t* custom_desc;
  zb_zcl_custom_cluster_cmd3_req_t cmd3;
  zb_bool_t processed = ZB_TRUE;
  zb_zcl_parsed_hdr_t cmd_info;
  zb_zcl_custom_addr_t main_addr;

  ZB_ZCL_COPY_PARSED_HEADER(param, &cmd_info);

  ZB_ASSERT(ZB_ZCL_CLUSTER_ID_CUSTOM == cmd_info.cluster_id);
  ZB_ASSERT(ZB_ZCL_FRAME_DIRECTION_TO_SRV == cmd_info.cmd_direction);

  main_addr.src_addr = ZB_ZCL_PARSED_HDR_SHORT_DATA(&cmd_info).source.u.short_addr;
  main_addr.src_endpoint             = ZB_ZCL_PARSED_HDR_SHORT_DATA(&cmd_info).src_endpoint;
  main_addr.dst_endpoint             = ZB_ZCL_PARSED_HDR_SHORT_DATA(&cmd_info).dst_endpoint;
  main_addr.cmd_id                   = cmd_info.cmd_id;
  main_addr.seq_number               = cmd_info.seq_number;
  main_addr.disable_default_response = (zb_bool_t)cmd_info.disable_default_response;
  main_addr.profile_id               = cmd_info.profile_id;

  custom_desc = zb_zcl_get_attr_desc_a(main_addr.dst_endpoint, ZB_ZCL_CLUSTER_ID_CUSTOM, ZB_ZCL_CLUSTER_SERVER_ROLE, ZB_ZCL_CUSTOM_CLUSTER_ATTR_CHAR_STRING_ID);

  ZB_ASSERT(custom_desc != NULL);

  cmd3 = *(zb_zcl_custom_cluster_cmd3_req_t*)custom_desc->data_p;

  TRACE_MSG(TRACE_ZCL1, "cmd3 is %i", (FMT__H, cmd3));

  TRACE_MSG(TRACE_ZCL1, "> zb_zcl_process_custom_specific_commands: param %d, cmd %d", (FMT__H_H, param, cmd_info.cmd_id));
  
  switch (main_addr.cmd_id)
  {
  case ZB_ZCL_CUSTOM_CLUSTER_CMD3_ID:
    /* code */
    custom_cmd_handler(param, &main_addr);
    TRACE_MSG(TRACE_ZCL3, "Processed custom command", (FMT__0));
    break;
  
  default:
    processed = ZB_FALSE;
    break;
  }
  //LOG_DBG("received cmd with id: %i", main_addr.cmd_id);

  TRACE_MSG(TRACE_ZCL1, "< zb_zcl_process_modbus_specific_commands: processed %d", (FMT__D, processed));

  return processed;
}


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
  return zb_zcl_process_custom_specific_commands(param);
}

zb_ret_t check_value_custom_server(zb_uint16_t attr_id, zb_uint8_t endpoint, zb_uint8_t *value)
{
  zb_ret_t ret = RET_OK;
  ZVUNUSED(endpoint);
  //ZVUNUSED(attr_id);

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
  zb_zcl_add_cluster_handlers(ZB_ZCL_CLUSTER_ID_CUSTOM,
                              ZB_ZCL_CLUSTER_SERVER_ROLE,
                              check_value_custom_server,
                              (zb_zcl_cluster_write_attr_hook_t)NULL,
                              zb_zcl_process_custom_specific_commands_srv);
}