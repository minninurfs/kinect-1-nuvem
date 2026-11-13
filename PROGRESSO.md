# Kinect 1 — nuvem de pontos ao vivo (notas técnicas)

Última atualização: 09/11/2026

## O que é
Programa para Mac (Apple Silicon) que usa o Kinect do Xbox 360 (modelo 1473) como "câmera"
de nuvem de pontos, com efeitos, gravação e saída Syphon para o OBS. Pensado para
ser uma câmera extra em lives e projeções.

## Arquivos
- `kinect_cloud.c` — o visualizador (OpenGL/GLUT + libfreenect).
- `syphon_out.m` — ponte com o Syphon (publica a imagem "Kinect").
- `syphon/` — código do Syphon-Framework (BSD, ver `syphon/Syphon/License.txt`), sem a parte Metal.
- `build.sh` — compila tudo: `./build.sh` e depois `./kinect_cloud`.
- `kinect_snap.c` — teste simples que salva um quadro de cor e um de profundidade.

Requisitos: Command Line Tools (`xcode-select --install`), `brew install libfreenect`, `ffmpeg` (gravação).

## Atalhos
| Tecla | Ação |
|---|---|
| E ou C | efeito: imagem real / profundidade / holograma / partículas / visão noturna / mapa limão / mapa ciano / nuvem com aura |
| , . 0 | exposição: escurece / clareia / normal (vale em todos os efeitos, ±3 pontos) |
| B | remover fundo (conta 5 s, saia da frente); B de novo desliga |
| V | grava/para H.264 .mp4, fundo preto |
| T | grava/para HEVC .mov, fundo transparente |
| P | salva quadro 3D .ply (Blender) |
| A | balanço automático da câmera |
| [ ] | distância máxima (corta o fundo) |
| + - | tamanho do ponto |
| arrastar / botão direito / Z X | girar / zoom |
| O | esconde a interface (janela limpa) |
| K | reconecta o Kinect |
| H | ajuda na tela |
| R / F / Q | vista original / tela cheia / sair |

Vídeos e .ply vão para `~/Documents/kinect`.

## Saída Syphon (OBS)
OBS → Fontes → + → Syphon Client → "[kinect_cloud] Kinect" → marcar Allow Transparency.
1920×1080, 30 fps, com alfa, sem a interface. Testado com um cliente Syphon próprio;
**ainda não testado no OBS de verdade**.

## Visão noturna (09/11)
Usa a câmera infravermelha: enxerga no escuro total, porque a "luz" é o padrão de pontinhos
que o próprio Kinect projeta. Imagem verde, com brilho automático e compensação de distância.
O Kinect 1 não manda cor e infravermelho ao mesmo tempo, mas a imagem pode ser trocada com tudo
rodando: ao entrar ou sair desse efeito só o fluxo de imagem é religado. A janela não fecha e o
Syphon, a gravação e o fundo removido continuam.

## Profundidade e cor
A profundidade fica sempre em milímetros (`FREENECT_DEPTH_MM`), do ponto de vista da câmera
infravermelha. O alinhamento com a cor é feito no programa (`register_color`), com a calibração
do próprio Kinect (`freenect_copy_registration`) e a mesma conta do libfreenect. Pontos que a
câmera colorida não enxerga (atrás de algo) não são desenhados nos efeitos que usam cor.

## Efeitos de mapa e aura (09/11)
Efeitos 5, 6 e 7 (`--efeito 5|6|7`):
- **Mapa limão**: fundo amarelo-limão estourando para o branco nas bordas, corpo grande e borrado do
  azul escuro ao verde, e uma cópia pequena branca da silhueta na frente.
- **Mapa ciano**: mesma ideia em ciano/azul/roxo, com fundo preto fora da área captada.
- **Nuvem com aura**: corpo cinza com relevo (sombreado por profundidade suavizada) e partículas
  vermelhas que nascem no contorno e sobem em fios.
Os três separam a "pessoa" da sala sozinhos (bloco de profundidade contínua, grande, perto e no
centro), ou usam o recorte do B se ele estiver ligado. A exposição (, . 0) vale neles também.
Os mapas são imagem 2D (não giram com o mouse). Ainda a ajustar com o Kinect em pé numa cena
real: posição/tamanho da cópia pequena, o degradê do corpo grande e a densidade/forma dos fios.

## Coisas aprendidas
- Modelo 1473: abrir só a câmera (`FREENECT_DEVICE_CAMERA`); o áudio pede firmware que não temos.
- A conexão USB do Kinect às vezes cai; o programa reconecta sozinho após 2 s sem imagem.
  Se nem isso resolver: desligar e religar tomada e USB. Ligar direto no Mac, sem hub.
- Muitas aberturas/fechamentos seguidos deixam o Kinect travado (precisa religar).
- Trocar o modo de PROFUNDIDADE (alinhada ↔ mm) no mesmo processo faz ela vir zerada, mesmo
  fechando e reabrindo o Kinect. Trocar só a imagem (cor ↔ infravermelho) funciona.
- Abrir o Kinect logo depois de fechar às vezes leva ~10 s (`freenect_open_device`). Por isso
  a reconexão roda em segundo plano e a janela não congela; a gravação pula esse trecho.
- A câmera colorida não enxerga no escuro: com a luz apagada os efeitos "imagem real" e
  "partículas" ficam pretos. Use a visão noturna, profundidade ou holograma.
- Fechar o programa à força deixa o Kinect "preso" na próxima abertura; agora Ctrl+C e kill
  fecham direito.
- O SDK oficial do Syphon (v5) é só Intel; por isso compilamos do código-fonte.
- Consumo no M1 Max: ~2% da máquina (~3% com o OBS recebendo pelo Syphon).

## Opções de linha de comando
`--efeito 0-7`, `--exposicao EV`, `--corte MM`, `--ponto N`, `--tela-cheia`, `--sem-interface`,
`--fundo`, `--gravar` / `--gravar-alfa`, `--segundos N` e, para testes, `--trocar-efeito N`, `--reconectar-em N`.

## Próximos passos possíveis
- Testar a fonte Syphon no OBS.
- Mais efeitos / ajustes das partículas; gravar dado bruto e escolher o visual depois.
