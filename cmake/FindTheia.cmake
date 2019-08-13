# - Try to find OPENMESH_VEC (the simple version including only the vector class)
# Once done this will define
#  
# OPENMESH_VEC_FOUND           - system has OPENMESH_vec
# OPENMESH_VEC_INCLUDE_DIR - theOPENMESH include directory

# IF (OPENMESH_INCLUDE_DIR)
 # Already in cache, be silent
# SET(OPENMESH_FIND_QUIETLY TRUE)
# ENDIF (OPENMESH_INCLUDE_DIR)


FIND_PATH(THEIA_INCLUDE_DIR theia/theia.h 
	PATHS "/media/desktop/1TB/documents/TheiaSfM-0.5/include" )
	
if( THEIA_INCLUDE_DIR )
    set( THEIA_FOUND TRUE )
else( THEIA_INCLUDE_DIR )
    set( THEIA_FOUND FALSE )
endif( THEIA_INCLUDE_DIR )