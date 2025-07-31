#!/bin/bash

# Script para configurar el entorno de nRF Connect SDK
# Uso: source setup_ncs_env.sh

# Colores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}🔧 Configurando entorno nRF Connect SDK...${NC}"

# Buscar automáticamente la instalación del SDK
NCS_BASE_PATHS=(
    "/opt/nordic/ncs"
    "$HOME/ncs"
    "$HOME/nordic/ncs"
    "/usr/local/nordic/ncs"
)

NCS_VERSIONS=(
    "v2.9.2"
    "v2.9.1"
    "v2.9.0"
    "v2.8.0"
    "v2.7.0"
    "v2.6.1"
    "v2.6.0"
)

# Función para buscar SDK
find_ncs_sdk() {
    for base_path in "${NCS_BASE_PATHS[@]}"; do
        if [ -d "$base_path" ]; then
            echo -e "${YELLOW}📂 Buscando en: $base_path${NC}"
            
            for version in "${NCS_VERSIONS[@]}"; do
                if [ -d "$base_path/$version" ]; then
                    if [ -f "$base_path/$version/zephyr/zephyr-env.sh" ]; then
                        echo -e "${GREEN}✅ Encontrado SDK: $base_path/$version${NC}"
                        NCS_PATH="$base_path/$version"
                        return 0
                    fi
                fi
            done
            
            # Buscar cualquier directorio que contenga zephyr
            for dir in "$base_path"/*; do
                if [ -d "$dir/zephyr" ] && [ -f "$dir/zephyr/zephyr-env.sh" ]; then
                    echo -e "${GREEN}✅ Encontrado SDK: $dir${NC}"
                    NCS_PATH="$dir"
                    return 0
                fi
            done
        fi
    done
    return 1
}

# Buscar el SDK
if ! find_ncs_sdk; then
    echo -e "${RED}❌ No se encontró ninguna instalación del nRF Connect SDK${NC}"
    echo -e "${YELLOW}📍 Rutas buscadas:${NC}"
    for path in "${NCS_BASE_PATHS[@]}"; do
        for version in "${NCS_VERSIONS[@]}"; do
            echo "   - $path/$version"
        done
    done
    echo -e "${YELLOW}💡 Instala el SDK o configura manualmente:${NC}"
    echo "   export NCS_PATH=/path/to/your/ncs"
    return 1
fi

# Configurar variables de entorno
export ZEPHYR_BASE="$NCS_PATH/zephyr"
echo -e "${BLUE}📍 ZEPHYR_BASE: $ZEPHYR_BASE${NC}"

# Buscar toolchains automáticamente
TOOLCHAIN_BASE="$NCS_PATH/../toolchains"
if [ ! -d "$TOOLCHAIN_BASE" ]; then
    TOOLCHAIN_BASE="$NCS_PATH/toolchains"
fi

if [ -d "$TOOLCHAIN_BASE" ]; then
    echo -e "${YELLOW}🔍 Buscando toolchains en: $TOOLCHAIN_BASE${NC}"
    
    # Buscar CMake
    CMAKE_PATH=$(find "$TOOLCHAIN_BASE" -name "cmake" -type f 2>/dev/null | head -1)
    if [ -n "$CMAKE_PATH" ]; then
        CMAKE_DIR=$(dirname "$CMAKE_PATH")
        export PATH="$CMAKE_DIR:$PATH"
        echo -e "${GREEN}🔨 CMAKE encontrado: $CMAKE_PATH${NC}"
    fi
    
    # Buscar ninja y otras herramientas
    TOOLS_PATH=$(find "$TOOLCHAIN_BASE" -name "ninja" -type f 2>/dev/null | head -1)
    if [ -n "$TOOLS_PATH" ]; then
        TOOLS_DIR=$(dirname "$TOOLS_PATH")
        export PATH="$TOOLS_DIR:$PATH"
        echo -e "${GREEN}⚡ NINJA encontrado: $TOOLS_PATH${NC}"
    fi
    
    # Buscar arm-none-eabi-gcc
    GCC_PATH=$(find "$TOOLCHAIN_BASE" -name "arm-none-eabi-gcc" -type f 2>/dev/null | head -1)
    if [ -n "$GCC_PATH" ]; then
        GCC_DIR=$(dirname "$GCC_PATH")
        export PATH="$GCC_DIR:$PATH"
        echo -e "${GREEN}🛠️  ARM-GCC encontrado: $GCC_PATH${NC}"
    fi
else
    echo -e "${YELLOW}⚠️  No se encontró directorio de toolchains${NC}"
fi

# Activar el entorno de Zephyr
if [ -f "$ZEPHYR_BASE/zephyr-env.sh" ]; then
    echo -e "${BLUE}🚀 Activando entorno Zephyr...${NC}"
    source "$ZEPHYR_BASE/zephyr-env.sh"
    echo -e "${GREEN}✅ Entorno Zephyr activado${NC}"
else
    echo -e "${RED}❌ No se encontró zephyr-env.sh en $ZEPHYR_BASE${NC}"
    return 1
fi

# Verificar herramientas
echo -e "\n${BLUE}🔍 Verificando herramientas:${NC}"

if command -v cmake >/dev/null 2>&1; then
    CMAKE_VERSION=$(cmake --version | head -1)
    echo -e "${GREEN}🔨 CMAKE: $(which cmake) - $CMAKE_VERSION${NC}"
else
    echo -e "${RED}❌ CMAKE no encontrado${NC}"
fi

if command -v ninja >/dev/null 2>&1; then
    echo -e "${GREEN}⚡ NINJA: $(which ninja)${NC}"
else
    echo -e "${RED}❌ NINJA no encontrado${NC}"
fi

if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    GCC_VERSION=$(arm-none-eabi-gcc --version | head -1)
    echo -e "${GREEN}🛠️  ARM-GCC: $GCC_VERSION${NC}"
else
    echo -e "${RED}❌ ARM-GCC no encontrado${NC}"
fi

if command -v west >/dev/null 2>&1; then
    WEST_VERSION=$(west --version)
    echo -e "${GREEN}🌐 WEST: $WEST_VERSION${NC}"
else
    echo -e "${RED}❌ WEST no encontrado${NC}"
fi

# Verificar si estamos en un workspace west
if west topdir >/dev/null 2>&1; then
    WORKSPACE=$(west topdir)
    echo -e "${GREEN}📁 WORKSPACE: $WORKSPACE${NC}"
else
    echo -e "${YELLOW}⚠️  No estás en un workspace west válido${NC}"
fi

echo -e "\n${GREEN}✅ Entorno nRF Connect SDK configurado correctamente${NC}"
echo -e "${BLUE}📖 Comandos disponibles:${NC}"
echo -e "  ${YELLOW}west build -b thingy91x/nrf5340/cpuapp --pristine${NC}  # Compilar proyecto"
echo -e "  ${YELLOW}west flash${NC}                                         # Flashear aplicación"
echo -e "  ${YELLOW}west flash --domain ipc_radio${NC}                      # Flashear network core"
echo -e "  ${YELLOW}west flash --erase${NC}                                 # Borrar y flashear"

# Guardar configuración para sesiones futuras
PROFILE_FILE=""
if [ -f "$HOME/.zshrc" ]; then
    PROFILE_FILE="$HOME/.zshrc"
elif [ -f "$HOME/.bashrc" ]; then
    PROFILE_FILE="$HOME/.bashrc"
elif [ -f "$HOME/.bash_profile" ]; then
    PROFILE_FILE="$HOME/.bash_profile"
fi

if [ -n "$PROFILE_FILE" ]; then
    echo -e "\n${BLUE}💾 ¿Deseas hacer esta configuración permanente?${NC}"
    echo -e "Esto agregará las variables de entorno a $PROFILE_FILE"
    echo -e "${YELLOW}Ejecuta: echo 'source $(pwd)/setup_ncs_env.sh' >> $PROFILE_FILE${NC}"
fi
