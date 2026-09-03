# MaixCDK main 组件。头文件在仓库根目录 include/（本文件上三级）。
list(APPEND ADD_INCLUDE "${CMAKE_CURRENT_LIST_DIR}/../../../include")
append_srcs_dir(ADD_SRCS "src")
list(APPEND ADD_REQUIREMENTS basic peripheral)
register_component()
