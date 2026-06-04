#include "rayTracing.h"
#include "lighting.h"
#include "geometry.h"

void RayTracer(const CenaProcessada& dados, const Camera& cam, const SceneData& scene){
    std::vector<std::array<int, 3>> image_buffer(cam.hres * cam.vres);
    for(int j = 0; j < cam.vres; j ++){
        cerr<<"Linha "<<j<<'/'<<cam.vres<<'\r'<<flush;
        for(int i = 0; i < cam.hres; i++){
            Vetor ray_dir = cam.getRayDirection(i, j);
            double closest_t = infinity();
            const ObjectData* hit_obj = nullptr;
            Vetor hit_normal;
            for(const auto& objeto : dados.valid_objects){
                HitResult hr = intersect_object(objeto, cam.C, ray_dir);
                if(hr.t < closest_t){
                    closest_t  = hr.t;
                    hit_obj    = &objeto;
                    hit_normal = hr.normal;
                }
            }

            int r = 0, g = 0, b = 0;
            if(hit_obj){
                Ponto P = cam.C + (ray_dir * closest_t);
                auto [cor_r, cor_g, cor_b] = calcular_cor_phong(
                    P, hit_normal, ray_dir, *hit_obj, scene.globalLight.color, scene.camera.lookfrom, scene.lightList, dados.valid_objects, intersect_object);

                r = (int)(255.999 * cor_r);
                g = (int)(255.999 * cor_g);
                b = (int)(255.999 * cor_b);
            }

            int pixel_index = j * cam.hres + i;
            image_buffer[pixel_index] = {r, g, b};
        }
    }

    for (const auto& pixel : image_buffer) {
        cout << pixel[0] << ' ' << pixel[1] << ' ' << pixel[2] << '\n';
    }
}