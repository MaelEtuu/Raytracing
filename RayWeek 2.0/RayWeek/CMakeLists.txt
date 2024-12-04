# CMakeList.txt : projet CMake pour RayWeek, incluez la source et définissez
# la logique spécifique au projet ici.
#

# Ajoutez une source à l'exécutable de ce projet.
add_executable (RayWeek "RayWeek.cpp" "RayWeek.h" "color.h" "vec3.h" "ray.h" "hittable.h" "sphere.h" "hittable_list.h" "rtweekend.h" "interval.h" "camera.h" "material.h")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET RayWeek PROPERTY CXX_STANDARD 20)
endif()

# TODO: Ajoutez des tests et installez des cibles si nécessaire.
