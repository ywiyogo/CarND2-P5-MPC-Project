import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import click

@click.command()
@click.argument("output")


def main(output):
    print("Output file: ", output)
    df = pd.read_table(output, sep='\t')
    print(df.head(n=5))

    fig = plt.figure(figsize=(16, 8))

    fig.suptitle('MPC - Different dt and Cost Weight', fontsize=18, fontweight='bold')
    ax1 = fig.add_subplot(221)
    fig.subplots_adjust(top=0.85)
    ax1.set_title('CTE', fontsize=18, fontweight='bold')
    ax1.set_xlabel('time')
    ax1.set_ylabel('CTE')
    ax1.plot(df['cte_N10_dt005_wd500'])
    ax1.plot(df['cte_N8_dt005_wd500'])
    ax1.plot(df['cte_N10_dt005_wd500_ad'])
    ax1.legend(loc='upper right')
    ax1.grid(linestyle=":")

    ax2 = fig.add_subplot(222)
    ax2.set_title('Steering', fontsize=18, fontweight='bold')
    ax2.set_xlabel('time')
    ax2.set_ylabel('steer value')

    ax2.plot(df['steering_N10_dt005_wd500'])
    ax2.plot(df['steering_N8_dt005_wd500'])
    ax2.plot(df['steering_N10_dt005_wd500_ad'])

    ax2.legend(loc='upper right')
    ax2.grid(linestyle=":")

    ax3 = fig.add_subplot(223)
    ax3.set_title('Error psi', fontsize=18, fontweight='bold')
    ax3.set_xlabel('time')
    ax3.set_ylabel('epsi')
    ax3.plot(df['epsi_N10_dt005_wd500'])
    ax3.plot(df['epsi_N8_dt005_wd500'])
    ax3.plot(df['epsi_N10_dt005_wd500_ad'])

    ax3.grid(linestyle=":")
    ax3.legend(loc='upper right')

    ax4 = fig.add_subplot(224)
    ax4.set_title('Cost', fontsize=18, fontweight='bold')
    ax4.set_xlabel('time')
    ax4.set_ylabel('epsi')
    ax4.plot(df['cost_N10_dt005_wd500'])
    ax4.plot(df['cost_N8_dt005_wd500'])
    ax4.plot(df['cost_N10_dt005_wd500_ad'])


    ax4.grid(linestyle=":")

    fig.tight_layout()
    ax4.legend(loc='upper right')
    plt.savefig('experiments.png')

    plt.show()

if __name__ == '__main__':
    main()