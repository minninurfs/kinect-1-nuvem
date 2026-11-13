# Kinect 1 — nuvem de pontos ao vivo

Transforma o Kinect do Xbox 360 (modelo 1473) numa câmera de nuvem de pontos para Mac (Apple Silicon), com efeitos, gravação e saída Syphon para o OBS.

## Instalar
1. `xcode-select --install`
2. `brew install libfreenect ffmpeg`
3. Nesta pasta: `./build.sh`
4. Rodar: `./kinect_cloud` (aperte **H** para ver os atalhos)

## No OBS
Fontes → + → Syphon Client → "[kinect_cloud] Kinect" → marcar *Allow Transparency*.

Os detalhes, atalhos e anotações estão em [PROGRESSO.md](PROGRESSO.md).
O código do Syphon em `syphon/` é de terceiros, com licença BSD (`syphon/Syphon/License.txt`).

Licença: MIT (arquivo `LICENSE`), exceto `syphon/`.
