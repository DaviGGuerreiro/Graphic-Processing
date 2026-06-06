#include "rayTracing.h"
#include "lighting.h"
#include "geometry.h"

namespace {
    ColorData cor_global;
    std::vector<LightData> LightList;
    std::vector<ObjectData> objetos_validos;
    Ponto CameraPos;

    bool notTIR(double n_i, double n_t, const Vetor& N, const Vetor& d_neg) {
        double cos_i = N.dot(d_neg);           // cos(θi)
        double ratio = n_i / n_t;
        double discriminant = 1.0 - (ratio * ratio) * (1.0 - cos_i * cos_i);
        return discriminant >= 0.0;            // false = TIR ocorre
    }

    Vetor refractDirection(double n_i, double n_t, const Vetor& N, const Vetor& d_neg) {
        // d_neg = -d (direção invertida do raio incidente)
        double ratio   = n_i / n_t;
        double cos_i   = N.dot(d_neg);                          // cos(θi)
        double disc    = 1.0 - ratio * ratio * (1.0 - cos_i * cos_i);
        double cos_t   = std::sqrt(disc);                       // cos(θt)

        // T = (n_i/n_t)*d + (n_i/n_t * cos_i - cos_t)*N
        // lembrando que d = -d_neg
        Vetor d = d_neg * (-1.0);
        return d * ratio + N * (ratio * cos_i - cos_t);
    }

    std::array<double, 3> RayTracer(const Vetor& direcao, const Ponto& origem , int iteracao){
        std::array<double, 3> arr = {0, 0, 0};
        if(iteracao >= 10){
            return arr;
        }
        double closest_t = infinity();
        const ObjectData* hit_obj = nullptr;
        Vetor hit_normal;
        for(const auto& objeto : objetos_validos){
            HitResult hr = intersect_object(objeto, origem, direcao);
            if(hr.t < closest_t){
                closest_t  = hr.t;
                hit_obj    = &objeto;
                hit_normal = hr.normal;
            }
        }
        if(hit_obj){
            Ponto P = origem + (direcao * closest_t);
            auto [cor_r, cor_g, cor_b] = calcular_cor_phong(
                P, hit_normal, direcao, *hit_obj, cor_global, CameraPos, LightList, objetos_validos, intersect_object);
            arr[0] = cor_r;
            arr[1] = cor_g;
            arr[2] = cor_b;
            MaterialData Material = hit_obj->material;
            if(Material.kr.r > 0.0 || Material.kr.g > 0.0 || Material.kr.b > 0.0){
                Ponto P_refletido = P + (hit_normal * 1e-4);
                Vetor refletido = direcao - hit_normal * (2.0 * direcao.dot(hit_normal));
                std::array<double, 3> cor_refletido = RayTracer(refletido, P_refletido, iteracao + 1);
                arr[0] += Material.kr.r * cor_refletido[0];
                arr[1] += Material.kr.g * cor_refletido[1];
                arr[2] += Material.kr.b * cor_refletido[2];
            }
            if(Material.kt.r > 0 || Material.kt.g > 0 || Material.kt.b > 0){
                bool entrando = direcao.dot(hit_normal) < 0.0;
                double n_i, n_t;
                Vetor N_refr = hit_normal;
                if(entrando){
                    n_i = 1.0; n_t = Material.ni;
                }
                else{
                    n_i = Material.ni; n_t = 1.0;
                    N_refr = hit_normal * (-1.0);
                }
                if(notTIR(n_i, n_t, N_refr, -direcao)){
                    Ponto P_refratado = P - (N_refr * 1e-4);
                    Vetor refratado = refractDirection(n_i, n_t, N_refr, -direcao);
                    std::array<double, 3> cor_refratado = RayTracer(refratado, P_refratado, iteracao + 1);
                    arr[0] += Material.kt.r * cor_refratado[0];  // soma em float
                    arr[1] += Material.kt.g * cor_refratado[1];
                    arr[2] += Material.kt.b * cor_refratado[2];
                }
            }
            arr[0] = std::min(1.0, arr[0]);
            arr[1] = std::min(1.0, arr[1]);
            arr[2] = std::min(1.0, arr[2]);
        }
        return arr;
    }
}

void Trace(const CenaProcessada& dados, const Camera& cam, const SceneData& scene){
    std::vector<std::array<int, 3>> image_buffer(cam.hres * cam.vres);
    objetos_validos = dados.valid_objects;
    LightList = scene.lightList;
    cor_global = scene.globalLight.color;
    CameraPos = scene.camera.lookfrom;
    for(int j = 0; j < cam.vres; j ++){
        cerr<<"Linha "<<j<<'/'<<cam.vres<<'\r'<<flush;
        for(int i = 0; i < cam.hres; i++){
            int pixel_index = j * cam.hres + i;
            Vetor ray_dir = cam.getRayDirection(i, j);
            auto pixel_desenhado = RayTracer(ray_dir, CameraPos, 1);
            image_buffer[pixel_index] = {
                (int)(255.999 * pixel_desenhado[0]),
                (int)(255.999 * pixel_desenhado[1]),
                (int)(255.999 * pixel_desenhado[2])
            };
        }
    }
    for (const auto& pixel : image_buffer) {
        cout << pixel[0] << ' ' << pixel[1] << ' ' << pixel[2] << '\n';
    }
}