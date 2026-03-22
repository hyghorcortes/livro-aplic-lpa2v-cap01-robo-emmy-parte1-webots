# Exemplo de Aplicação: Robô Emmy – Parte 1 (Webots)

Implementação em **Webots** do exemplo **“Exemplo de Aplicação: Exemplo do Robô Emmy – Parte 1”**, associado ao **Capítulo 1 – Para-analisadores** do livro *Aplicações de LPA2v*.

Este repositório organiza o projeto completo do exemplo, incluindo:

- controlador principal em Python (`drive_my_robot.py`);
- mundo do Webots (`.wbt`);
- arquivos auxiliares do projeto;
- documentação de uso e publicação no GitHub.

## Objetivo do exemplo

Este exemplo mostra uma aplicação prática da **LPA2v** em robótica móvel autônoma. A proposta é conectar:

1. construção de evidências sensoriais `(μ, λ)`;
2. análise dos estados lógicos pelo **Para-Analisador**;
3. escolha de rotinas motoras inspiradas no robô Emmy I;
4. execução em ambiente simulado no **Webots**.

A implementação atual é uma **releitura didática em Webots**, preservando o núcleo lógico do robô Emmy I e adicionando camadas complementares de robustez, como `escape`, `cooldown`, detecção de travamento e controle contínuo de fallback.

## Estrutura do repositório

```text
.
├── controllers/
│   └── drive_my_robot/
│       └── drive_my_robot.py
├── worlds/
│   ├── empty_emmy_v15_fov180.wbt
│   ├── .empty_emmy_v15.wbproj
│   ├── .empty_emmy_v15_fov180.wbproj
│   └── ...
├── libraries/
├── plugins/
├── protos/
├── docs/
├── assets/
├── .github/
├── README.md
├── LICENSE
├── CITATION.cff
├── CONTRIBUTING.md
├── CODE_OF_CONDUCT.md
├── SECURITY.md
├── CHANGELOG.md
├── .gitignore
└── IMPORTAR_NO_GITHUB.md
```

## Arquivo principal

O ponto central do projeto é:

```text
controllers/drive_my_robot/drive_my_robot.py
```

Esse controlador:

- usa uma partição em **12 estados lógicos**;
- calcula `Gc = μ - λ` e `Gct = μ + λ - 1`;
- associa estados extremos e não extremos a rotinas discretas do artigo;
- preserva uma camada de segurança acima das rotinas lógicas.

## Mundo principal

O mundo principal do exemplo é:

```text
worlds/empty_emmy_v15_fov180.wbt
```

A primeira linha do arquivo indica compatibilidade com:

```text
#VRML_SIM R2025a utf8
```

## Como executar

### 1. Abra o projeto no Webots
Abra o arquivo:

```text
worlds/empty_emmy_v15_fov180.wbt
```

### 2. Verifique o controlador
O robô no mundo já está configurado com o controlador:

```text
controller "drive_my_robot"
```

### 3. Execute a simulação
Ao iniciar a simulação, o controlador Python será carregado e o console exibirá mensagens de debug com:

- leituras filtradas dos sensores;
- valores de `μ` e `λ`;
- estado lógico atual;
- `Gc` e `Gct`;
- rotina nominal e rotina executada;
- status de `escape`, `avoid`, `stuck` e sinalização.

## Dependências

- **Webots R2025a** ou compatível com o mundo `.wbt` fornecido;
- Python embutido no Webots para execução do controlador;
- nenhum pacote externo do `pip` é exigido neste exemplo.

## Organização dentro da coleção do livro

Este repositório foi pensado para ser **um repositório individual de um conjunto maior de exemplos do livro**. A convenção sugerida é:

```text
livro-lpa2v-capXX-nome-do-exemplo-parteY-tecnologia
```

Nome sugerido deste repositório:

```text
livro-lpa2v-cap01-robo-emmy-parte1-webots
```

## Publicação no GitHub

Os arquivos de apoio à publicação já estão incluídos:

- `LICENSE`
- `README.md`
- `CITATION.cff`
- `CONTRIBUTING.md`
- `CODE_OF_CONDUCT.md`
- `SECURITY.md`

As instruções para subir o projeto ao GitHub estão em:

```text
IMPORTAR_NO_GITHUB.md
```

## Como citar

Use o arquivo `CITATION.cff` ou adapte a referência abaixo:

```bibtex
@software{miranda_cortes_santos_emmy_webots,
  author  = {Hyghor Miranda Côrtes and Paulo Santos},
  title   = {Exemplo de Aplicação: Robô Emmy - Parte 1 (Webots)},
  year    = {2026},
  version = {1.0.0}
}
```

## Licença

Este repositório está distribuído sob a licença **MIT**.
