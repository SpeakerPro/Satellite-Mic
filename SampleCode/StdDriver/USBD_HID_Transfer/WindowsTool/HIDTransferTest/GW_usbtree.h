#pragma once

#define USB_DEVICE_NAME_MAX_LENGTH      255

typedef enum {
    USBTREE_SUCCESSFUL = 0,
    USBTREE_EXECUTE_FAILED,
    USBTREE_LOG_OPEN_FAILED,
    USBTREE_MEMORY_ALLOCATE_FAILED
} USBTREE_ERROR_CODE_E;

typedef struct _USBTREE_ID_S
{
    INT     iVendor;
    INT     iProduct;
    CHAR    port_name[USB_DEVICE_NAME_MAX_LENGTH];
    CHAR    product_name[USB_DEVICE_NAME_MAX_LENGTH];
} USBTREE_ID_S;

INT UsbtreeReload(void);

BOOL UsbtreeSourceCheck(USBTREE_ID_S source, USBTREE_ID_S target);

void UsbtreeListShow(void);
