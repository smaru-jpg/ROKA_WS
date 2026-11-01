# Bash completion for the ROKA operator command.
# This file only completes words; it never launches ROS or robot hardware.

_roka_complete()
{
  local current profile command
  current="${COMP_WORDS[COMP_CWORD]}"
  profile="${COMP_WORDS[1]:-}"
  command="${COMP_WORDS[2]:-}"
  COMPREPLY=()

  local top_commands="drive manual mapping process review auto flat track mujoco vision sections verify handoff log help"
  local profile_commands="mapping process review auto check status replay view audit \
prepare prepare-cleaned blind-record mission1 mission2 mission3 mission5 arm-check arm-observe arm \
undo-review reset-review save-review terrain-mode terrain-undo terrain-reset terrain-save calibrate"
  profile_commands+=" terrain-commit terrain-cancel"
  local sections="section_1 section_2 section_3 section_4 section_5"

  if (( COMP_CWORD == 1 )); then
    COMPREPLY=( $(compgen -W "$top_commands" -- "$current") )
    return 0
  fi

  if [[ "$profile" == "mapping" ]]; then
    if (( COMP_CWORD == 2 )); then
      COMPREPLY=( $(compgen -W "$sections" -- "$current") )
    fi
    return 0
  fi

  if [[ "$profile" == "mujoco" ]]; then
    if (( COMP_CWORD == 2 )); then
      COMPREPLY=( $(compgen -W "mapping review" -- "$current") )
    elif [[ "${COMP_WORDS[2]:-}" == "mapping" && "$COMP_CWORD" -eq 3 ]]; then
      COMPREPLY=( $(compgen -W "lap01_2 fast_test01_2" -- "$current") )
    fi
    return 0
  fi

  if [[ "$profile" == "process" ]]; then
    if (( COMP_CWORD == 2 )); then
      COMPREPLY=( $(compgen -W "$sections" -- "$current") )
    elif (( COMP_CWORD == 3 )); then
      COMPREPLY=( $(compgen -W "all 2d 3d" -- "$current") )
    fi
    return 0
  fi

  if [[ "$profile" == "review" || "$profile" == "auto" ]]; then
    if (( COMP_CWORD == 2 )); then
      COMPREPLY=( $(compgen -W "$sections" -- "$current") )
    elif (( COMP_CWORD == 3 )); then
      COMPREPLY=( $(compgen -W "2d 3d" -- "$current") )
    elif [[ "$profile" == "auto" && "$COMP_CWORD" -eq 4 ]]; then
      if [[ "${COMP_WORDS[3]:-}" == "3d" ]]; then
        COMPREPLY=( $(compgen -W "recorded test full" -- "$current") )
      else
        COMPREPLY=( $(compgen -W "recorded test" -- "$current") )
      fi
    elif [[ "$profile" == "review" && "$COMP_CWORD" -eq 4 ]]; then
      COMPREPLY=( $(compgen -W "0.20 0.30 0.40" -- "$current") )
    fi
    return 0
  fi

  if [[ "$profile" != "flat" && "$profile" != "track" ]]; then
    if [[ "$profile" == "vision" && "$COMP_CWORD" -eq 2 ]]; then
      COMPREPLY=( $(compgen -W "spring summer autumn traffic" -- "$current") )
    fi
    if [[ "$profile" == "handoff" && "$COMP_CWORD" -eq 2 ]]; then
      COMPREPLY=( $(compgen -f -- "$current") )
    fi
    return 0
  fi

  if (( COMP_CWORD == 2 )); then
    COMPREPLY=( $(compgen -W "$profile_commands" -- "$current") )
    return 0
  fi

  case "$command:$COMP_CWORD" in
    mapping:3|process:3|check:3|status:3|replay:3|view:3|audit:3|auto:3|review:3|\
    prepare:3|prepare-cleaned:3)
      COMPREPLY=( $(compgen -W "$sections" -- "$current") )
      ;;
    blind-record:3|mission1:3)
      COMPREPLY=( $(compgen -W "section_1" -- "$current") )
      ;;
    mission2:3)
      COMPREPLY=( $(compgen -W "section_2" -- "$current") )
      ;;
    mission5:3)
      if [[ "$profile" == "track" ]]; then
        COMPREPLY=( $(compgen -W "0.60 0.80 1.00 1.20" -- "$current") )
      else
        COMPREPLY=( $(compgen -W "0.60" -- "$current") )
      fi
      ;;
    auto:4)
      if [[ "$profile" == "track" ]]; then
        COMPREPLY=( $(compgen -W "recorded test full" -- "$current") )
      else
        COMPREPLY=( $(compgen -W "recorded test" -- "$current") )
      fi
      ;;
    replay:4)
      COMPREPLY=( $(compgen -W "result process" -- "$current") )
      ;;
    replay:5)
      COMPREPLY=( $(compgen -W "0.25 0.5 1.0 2.0" -- "$current") )
      ;;
    terrain-mode:3)
      COMPREPLY=( $(compgen -W "possible blocked inspect" -- "$current") )
      ;;
    prepare:4|prepare-cleaned:4)
      COMPREPLY=( $(compgen -W "0.10 0.20 0.30" -- "$current") )
      ;;
    review:4)
      COMPREPLY=( $(compgen -W "0.20 0.30 0.40" -- "$current") )
      ;;
    arm-check:3)
      COMPREPLY=( $(compgen -W "hardware software" -- "$current") )
      ;;
    calibrate:3)
      if [[ "$profile" == "track" ]]; then
        COMPREPLY=( $(compgen -W "1 2 status" -- "$current") )
      fi
      ;;
    blind-record:4|mission1:4)
      COMPREPLY=( $(compgen -W "fog_1" -- "$current") )
      ;;
    blind-record:5)
      COMPREPLY=( $(compgen -W "20 30 40" -- "$current") )
      ;;
  esac
}

complete -F _roka_complete roka
