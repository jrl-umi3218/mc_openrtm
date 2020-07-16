set(BASE_JOINT_TYPE "free")
configure_file(JVRC1/main.in.wrl ${CMAKE_CURRENT_BINARY_DIR}/JVRC1/main.wrl)

set(BASE_JOINT_TYPE "fixed")
configure_file(JVRC1/main.in.wrl ${CMAKE_CURRENT_BINARY_DIR}/JVRC1/main_fixed.wrl)

install(FILES
    JVRC1/Map01_siri.jpg
    JVRC1/Map03_douF.jpg
    JVRC1/Map03_douR.jpg
    JVRC1/Map05_atamaF.jpg
    JVRC1/Map05_atamaR.jpg
    JVRC1/Map06_migikata.jpg
    JVRC1/Map07_migijowan.jpg
    JVRC1/Map08_migikawan.jpg
    JVRC1/Map09_migitekubi.jpg
    JVRC1/Map10_migitenohira.jpg
    JVRC1/Map12_migiisshisaki.jpg
    JVRC1/Map14_miginishisaki.jpg
    JVRC1/Map28_migimata.jpg
    JVRC1/Map29_migidaitai.jpg
    JVRC1/Map30_migisune.jpg
    JVRC1/Map31_migiashikubi.jpg
    JVRC1/Map32_migiashi.jpg
    JVRC1/chest.wrl
    JVRC1/head.wrl
    JVRC1/lankle.wrl
    JVRC1/lfoot.wrl
    JVRC1/lhip.wrl
    JVRC1/llarm.wrl
    JVRC1/llindex.wrl
    JVRC1/lllittle.wrl
    JVRC1/llthumb.wrl
    JVRC1/lpalm.wrl
    JVRC1/lshank.wrl
    JVRC1/lshoulder.wrl
    JVRC1/lthigh.wrl
    JVRC1/luarm.wrl
    JVRC1/luindex.wrl
    JVRC1/lulittle.wrl
    JVRC1/luthumb.wrl
    JVRC1/lwrist.wrl
    ${CMAKE_CURRENT_BINARY_DIR}/JVRC1/main.wrl
    ${CMAKE_CURRENT_BINARY_DIR}/JVRC1/main_fixed.wrl
    JVRC1/neck.wrl
    JVRC1/pelvis.wrl
    JVRC1/rankle.wrl
    JVRC1/rfoot.wrl
    JVRC1/rhip.wrl
    JVRC1/rlarm.wrl
    JVRC1/rlindex.wrl
    JVRC1/rllittle.wrl
    JVRC1/rlthumb.wrl
    JVRC1/rpalm.wrl
    JVRC1/rshank.wrl
    JVRC1/rshoulder.wrl
    JVRC1/rthigh.wrl
    JVRC1/ruarm.wrl
    JVRC1/ruindex.wrl
    JVRC1/rulittle.wrl
    JVRC1/ruthumb.wrl
    JVRC1/rwrist.wrl
    JVRC1/waist.wrl
    DESTINATION "${OPENHRP3_1_PREFIX}/share/OpenHRP-3.1/robot/JVRC1")

install(FILES
  environments/sim_mc_wall.wrl
  DESTINATION "${HRPSYS_BASE_PREFIX}/share/hrpsys/samples/environments")
