# =========================
# Compiler + flags
# =========================
CC      = gcc
CFLAGS  = -std=c99 -Wall -Werror -Iinclude -MMD -MP
#CFLAGS  = -std=c99 -Wall -g -Werror -Iinclude -MMD -MP
LDFLAGS = -lm

# =========================
# Project structure
# =========================
SRCDIR  = src
INCDIR  = include
OBJDIR  = build
TARGET  = $(OBJDIR)/main

# =========================
# Source / object lists
# =========================
SRCS := $(wildcard $(SRCDIR)/*.c)
OBJS := $(patsubst $(SRCDIR)/%.c,$(OBJDIR)/%.o,$(SRCS))
DEPS := $(OBJS:.o=.d)

# =========================
# Default target
# =========================
all: $(TARGET)

# =========================
# Link step
# =========================
$(TARGET): $(OBJS)
	@mkdir -p $(OBJDIR)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

# =========================
# Compile step
# =========================
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	@mkdir -p $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

# =========================
# Dependency includes
# =========================
-include $(DEPS)

# =========================
# Housekeeping
# =========================
clean:
	rm -rf $(OBJDIR)

.PHONY: all clean
