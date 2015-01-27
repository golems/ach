#compdef ach

function _ach {
    local curcontext="$curcontext" state line
    typeset -A opt_args

    _arguments \
        '1: :->command'\
        '*: :->args'

    case $state in
        command)
            local -a commands
            commands=(
                'mk: create a channel'
                'rm: remove a channel'
                'chmod: change channel permissions'
                'search: resolve channel host'
            )
            _describe -t commands 'command' commands && ret=0
            ;;
        args)
            case $words[2] in
                rm|search)
                    local K
                    if [ -d /dev/ach ]; then
                        K=`ls /dev/ach/ | sed -e 's!^ach-!!'`
                    fi
                    compadd `ls /dev/shm/achshm-* | sed -e 's!/dev/shm/achshm-!!'` "$K"
                    ;;
                mk)
                    local mks
                    mks=(
                        '-m size: message size'
                        '-n count: message count'
                        '-o mode: channel mode'
                        '-k: kernel channel'
                        '-u: user channel'
                        '-1: accept created channel'
                    )
                    _describe -t mks 'mk' mks && ret=0
                    ;;
                *)
                    ;;
            esac
    esac
}


_ach "$@"
